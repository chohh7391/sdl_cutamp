#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration as RclpyDuration # 이름 충돌을 피하기 위해 별칭 사용

from tamp_interfaces.msg import PlanStep
from tamp_interfaces.srv import Plan, Execute, SetTampEnv, MoveToTarget, SetTampCfg
from builtin_interfaces.msg import Duration
from simulation_interfaces.srv import GetEntityState
from std_srvs.srv import SetBool

from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from typing import Optional, List, Dict
import copy

from cutamp.algorithm import run_cutamp, setup_cutamp
from cutamp.config import TAMPConfiguration, validate_tamp_config
from cutamp.constraint_checker import ConstraintChecker
from cutamp.scripts.utils import (
    default_constraint_to_mult,
    default_constraint_to_tol,
    setup_logging,
    get_tetris_tuned_constraint_to_mult,
)
from cutamp.task_planning.base_structs import State
import logging
from cutamp.cost_reduction import CostReducer

from envs.utils import TAMPEnvManager
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
import numpy as np

from curobo.types.state import JointState as CuroboJointState
from curobo.types.math import Pose as CuroboPose
from curobo.types.base import TensorDeviceType


class TAMP:

    def __init__(
        self,
        config: TAMPConfiguration,
        use_tetris_tuned_weights: bool = None,
    ):
        validate_tamp_config(config)

        self.config = config

        self.env_manager = TAMPEnvManager()
        self.env = None

        setup_logging()

        self.use_tetris_tuned_weights = use_tetris_tuned_weights

        self.constraint_to_mult = (
            get_tetris_tuned_constraint_to_mult() if self.use_tetris_tuned_weights else default_constraint_to_mult.copy()
        )
        self.cost_reducer = CostReducer(self.constraint_to_mult)
        self.constraint_checker = ConstraintChecker(default_constraint_to_tol.copy())
        self._log = logging.getLogger(__name__)

        self.curobo_plan = None
        self.total_num_satisfying = None

        self.max_attempts = 3

        self.cmd_js_names = ["j1", "j2", "j3", "j4", "j5", "j6"]


    def update_config(self, config: TAMPConfiguration):
        self.config = config


    def update_env(
        self,
        name,
        poses: Dict[str, List[float]],
        movables: List[str],
        statics: List[str],
        ex_collision: List[str]
    ):
        self.env_manager.update_entities(
            poses=poses,
            movables=movables,
            statics=statics,
            ex_collision=ex_collision
        )
        self.env = self.env_manager.load_env(name)

    
    def plan(
        self,
        q_init: Optional[List[float]] = None,
        experiment_id: Optional[str] = None
    ):
        self.total_num_satisfying = 0

        if self.env is not None:
            for _ in range(self.max_attempts):
                env = copy.deepcopy(self.env)
                
                try:
                    self.curobo_plan, self.total_num_satisfying = run_cutamp(
                        env=env,
                        config=self.config,
                        cost_reducer=self.cost_reducer,
                        constraint_checker=self.constraint_checker,
                        q_init=q_init,
                        experiment_id=experiment_id
                    )
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")
                    self.total_num_satisfying = 0

                if self.total_num_satisfying > 0:
                    break
        else:
            raise ValueError("update_env is needed before plan")

        return self.curobo_plan, self.total_num_satisfying
    

    def motion_plan(
        self,
        q_init: List,
        ee_translation_goal: np.array,
        ee_orientation_goal: np.array,
    ):
        tensor_args = TensorDeviceType()

        _, _, timer, world = setup_cutamp(self.env, self.config, q_init)
        motion_gen = world.get_motion_gen(collision_activation_distance=self.config.world_activation_distance)
        if config.warmup_motion_gen:
            with timer.time("curobo_motion_gen_warmup"):
                motion_gen.warmup()

        plan_config = MotionGenPlanConfig(
            timeout=0.5, enable_finetune_trajopt=False, time_dilation_factor=config.time_dilation_factor
        )

        cu_js = CuroboJointState(
            position=tensor_args.to_device(q_init),
            velocity=tensor_args.to_device(q_init) * 0.0,
            acceleration=tensor_args.to_device(q_init) * 0.0,
            jerk=tensor_args.to_device(q_init) * 0.0,
            joint_names=self.cmd_js_names,
        )
        ik_goal = CuroboPose(
            position=tensor_args.to_device(ee_translation_goal),
            quaternion=tensor_args.to_device(ee_orientation_goal),
        )

        result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
        succ = result.success.item()
        if succ:
            cmd_plan = result.get_interpolated_plan().get_ordered_joint_state(self.cmd_js_names)
        else:
            return None
        
        return cmd_plan


        




class TAMPServer(Node):

    def __init__(self, tamp):
        
        super().__init__("tamp_server")

        self.tamp: TAMP = tamp
        self.get_logger().info(f"Initialize TAMP Module ...")

        self.reentrant_group = ReentrantCallbackGroup()

        # variables
        self.plan_to_execute = None
        self.plan_index = 0
        self.current_plan_step = 0
        self.execute_plan_timer = None
        self.move_to_target_timer = None

        # duration between plan_steps 
        self.is_waiting_after_step = False
        self.wait_start_time = None
        self.wait_duration = RclpyDuration(seconds=0.4)

        # commands
        self.arm_commands_publisher = self.create_publisher(JointState, "isaac_arm_commands", 10)
        self.gripper_commands_cli = self.create_client(SetBool, "isaac_gripper_commands", callback_group=self.reentrant_group)

        # subscription
        self.joint_states_subscription = self.create_subscription(JointState, "isaac_joint_states", self.joint_states_cb, 10)

        # services
        self.plan_srv = self.create_service(
            Plan, 'tamp_plan', self.tamp_plan_cb, 
            callback_group=self.reentrant_group
        )
        self.execute_srv = self.create_service(
            Execute, 'plan_execute', self.execute_plan_cb, 
            callback_group=self.reentrant_group
        )
        self.set_tamp_env_srv = self.create_service(
            SetTampEnv, 'set_tamp_env', self.set_tamp_env_cb, 
            callback_group=self.reentrant_group
        )
        self.set_tamp_cfg_srv = self.create_service(
            SetTampCfg, 'set_tamp_cfg', self.set_tamp_cfg_cb,
            callback_group=self.reentrant_group
        )
        self.move_to_target_srv = self.create_service(
            MoveToTarget, 'move_to_target', self.move_to_target_cb,
            callback_group=self.reentrant_group
        )
        
        # clients
        self.get_entity_state_cli = self.create_client(
            GetEntityState, 'get_entity_state',
            callback_group=self.reentrant_group
        )

        # robot states
        self.joint_states = JointState()
        self.arm_commands = JointState()


    async def set_tamp_env_cb(self, request, response):

        while not self.get_entity_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        env_name = request.env_name
        entities = request.entities
        movables = request.movables
        statics = request.statics
        ex_collision = request.ex_collision

        entities_states = {
            "poses": {},
            "movables": movables,
            "statics": statics,
            "ex_collision": ex_collision
        }

        for entity in entities:
            get_entity_state_request = GetEntityState.Request()
            get_entity_state_request.entity = "/World/" + entity

            get_entity_state_response = await self.get_entity_state_cli.call_async(get_entity_state_request)

            if get_entity_state_response.result.result == 1:

                response.success = True

                entity_pose = [
                    get_entity_state_response.state.pose.position.x,
                    get_entity_state_response.state.pose.position.y,
                    get_entity_state_response.state.pose.position.z + 0.01,
                    get_entity_state_response.state.pose.orientation.w,
                    get_entity_state_response.state.pose.orientation.x,
                    get_entity_state_response.state.pose.orientation.y,
                    get_entity_state_response.state.pose.orientation.z
                ]
                entities_states["poses"][entity] = entity_pose

        self.tamp.update_env(
            name=env_name,
            poses=entities_states["poses"],
            movables=entities_states["movables"],
            statics=entities_states["statics"],
            ex_collision=entities_states["ex_collision"]
        )

        return response
    
    def set_tamp_cfg_cb(self, request, response):

        if request.num_particles == 0:
            request.num_particles = 1024
        if request.robot == "":
            request.robot = "fr5"
        if request.grasp_dof == 0:
            request.grasp_dof = 6
        if request.approach == "":
            request.approach = "optimization"
        if request.num_resampling_attempts == 0:
            request.num_resampling_attempts = 100
        if request.num_opt_steps == 0:
            request.num_opt_steps = 1000
        if request.num_initial_plans == 0:
            request.num_initial_plans = 1
        if request.opt_viz_interval == 0:
            request.opt_viz_interval = 10

        config = TAMPConfiguration(
            num_particles=request.num_particles,
            robot=request.robot,
            grasp_dof=request.grasp_dof,
            approach=request.approach,
            num_resampling_attempts=request.num_resampling_attempts,
            num_opt_steps=request.num_opt_steps,
            max_loop_dur=None,
            optimize_soft_costs=request.optimize_soft_costs,
            soft_cost=None,
            num_initial_plans=request.num_initial_plans, # Changed from 1 to 30
            cache_subgraphs=None,
            curobo_plan=request.curobo_plan,
            enable_visualizer=request.enable_visualizer,
            opt_viz_interval=request.opt_viz_interval,
            viz_robot_mesh=request.viz_robot_mesh,
            enable_experiment_logging=request.enable_experiment_logging,
        )
        validate_tamp_config(config)

        self.tamp.update_config(config=config)

        response.success = True

        return response

    def joint_states_cb(self, msg):
        self.joint_states = msg

    def tamp_plan_cb(self, request, response):

        q_init = self.joint_states.position[:6].tolist()  # num_dof = 6

        try:
            curobo_plan, total_num_satisfying = self.tamp.plan(q_init, None)
            self.plan_to_execute = curobo_plan

            encoded_curobo_plan = self.process_plan(curobo_plan)
            response.plan_success = (total_num_satisfying > 0)
            response.curobo_plan = encoded_curobo_plan
            response.total_num_satisfying = total_num_satisfying

            self.get_logger().info(
                f"TAMP planning finished. Success: {response.plan_success}, "
                f"Satisfying particles: {total_num_satisfying}"
            )

        except Exception as e:
            self.get_logger().error(f"TAMP planning failed with exception: {e}")
            response.plan_success = False

        return response
    

    def process_plan(self, plan):

        processed_plan = []

        for plan_step in plan:
            value = PlanStep()

            if plan_step["type"] == "trajectory":
                value.type = 0 # TRAJECTORY
                traj = []

                dt = 0.05 # plan_step["dt"]
                duration = Duration()
                dt_nanosec = int(dt * 1e9)

                for i in range(plan_step["plan"].position.shape[0]):
                    traj_point = JointTrajectoryPoint()
                    
                    traj_point.positions = plan_step["plan"].position[i].tolist()
                    traj_point.velocities = plan_step["plan"].velocity[i].tolist()
                    traj_point.accelerations = plan_step["plan"].acceleration[i].tolist()
                    traj_point.time_from_start = duration
                    traj.append(traj_point)

                    duration.nanosec += dt_nanosec
                    
                    if duration.nanosec >= 1e9:
                        duration.sec += 1
                        duration.nanosec = int(duration.nanosec % 1e9)

                value.joint_trajectory.points = traj
                value.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
                value.joint_trajectory.joint_names = plan_step["plan"].joint_names

            elif plan_step["type"] == "gripper":
                value.type = 1 # GRIPPER
                value.action = plan_step["action"]
            else:
                raise ValueError("type of plan_step must be 'trajectory' or 'gripper'")
            
            processed_plan.append(value)

        return processed_plan
            

    def execute_plan_cb(self, request, response):

        if self.plan_to_execute and len(self.plan_to_execute) > 0:
            self.get_logger().info("Starting plan execution...")
            
            if self.execute_plan_timer is not None:
                self.execute_plan_timer.cancel()

            self.plan_index = 0
            self.current_plan_step = 0
            self.is_waiting_after_step = False # 실행 시작 시 대기 상태 초기화
            
            timer_period = 0.016 # 기본 타이머 주기
            for plan_part in self.plan_to_execute:
                if plan_part['type'] == 'trajectory':
                    timer_period = plan_part['dt']
                    break
            
            self.execute_plan_timer = self.create_timer(timer_period, self.execute_plan_timer_cb)
            response.execute_success = True
        else:
            self.get_logger().warn("No plan to execute.")
            response.execute_success = False


        return response


    def execute_plan_timer_cb(self):
        if self.is_waiting_after_step:
            if self.get_clock().now() - self.wait_start_time >= self.wait_duration:
                self.is_waiting_after_step = False
            else:
                return

        if not self.plan_to_execute or self.plan_index >= len(self.plan_to_execute):
            if self.execute_plan_timer is not None:
                self.execute_plan_timer.cancel()
                self.execute_plan_timer = None
            # self.plan_to_execute = None
            return

        current_plan_part = self.plan_to_execute[self.plan_index]
        plan_type = current_plan_part.get("type")
        
        step_finished = False

        if plan_type == "trajectory":
            plan_trajectory = current_plan_part["plan"]
            num_waypoints = plan_trajectory.position.shape[0]

            if self.current_plan_step < num_waypoints:
                self.arm_commands.header.stamp = self.get_clock().now().to_msg()
                self.arm_commands.name = plan_trajectory.joint_names
                self.arm_commands.position = plan_trajectory.position[self.current_plan_step].tolist()
                
                self.arm_commands_publisher.publish(self.arm_commands)
                
                self.current_plan_step += 1
            
            if self.current_plan_step >= num_waypoints:
                step_finished = True
                self.current_plan_step = 0 # 다음 궤적을 위해 초기화

        elif plan_type == "gripper":
            self.execute_gripper_action(current_plan_part)
            step_finished = True
            
        else:
            self.get_logger().error(f"Unknown plan type: {plan_type}")
            if self.execute_plan_timer is not None:
                self.execute_plan_timer.cancel()
            return

        if step_finished:
            self.plan_index += 1
            
            if self.plan_index < len(self.plan_to_execute):
                self.is_waiting_after_step = True
                self.wait_start_time = self.get_clock().now()


    def execute_gripper_action(self, plan_part):
        """
        Executes a gripper action by making a synchronous service call.
        """
        # Wait for the service to be available.
        if not self.gripper_commands_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gripper command service not available.')
            return

        request = SetBool.Request()
    
        if plan_part["action"] == "close":
            request.data = True
        else:  # open
            request.data = False

        # Use the synchronous service call. This blocks until the call is complete.
        # This is safe due to the MultiThreadedExecutor and the client's ReentrantCallbackGroup.
        response = self.gripper_commands_cli.call(request)

        if not response.success:
            self.get_logger().warn(f"Gripper action failed: {response.message}")


    def move_to_target_cb(self, request, response):

        self.cmd_plan = self.tamp.motion_plan(
            q_init=request.q_init,
            ee_translation_goal=np.array(request.target_position),
            ee_orientation_goal=np.array(request.target_orientation),
        )

        self.current_plan_step = 0
        timer_period = 0.016
        
        self.move_to_target_timer = self.create_timer(timer_period, self.move_to_target_timer_cb)
        response.success = True

        return response
    

    def move_to_target_timer_cb(self):

        if not self.cmd_plan or self.current_plan_step >= len(self.cmd_plan):
            if self.move_to_target_timer is not None:
                self.move_to_target_timer.cancel()
                self.move_to_target_timer = None
            return

        self.arm_commands.header.stamp = self.get_clock().now().to_msg()
        self.arm_commands.name = self.tamp.cmd_js_names
        self.arm_commands.position = self.cmd_plan[self.current_plan_step].position.tolist()
        self.arm_commands_publisher.publish(self.arm_commands)

        self.current_plan_step += 1
        

if __name__ == "__main__":

    rclpy.init()

    config = TAMPConfiguration(
        num_particles=1024,
        robot="fr5",
        grasp_dof=6,
        approach="optimization",
        num_resampling_attempts=100,
        num_opt_steps=1000,
        max_loop_dur=None,
        optimize_soft_costs=False,
        soft_cost=None,
        num_initial_plans=1, # Changed from 1 to 30
        cache_subgraphs=None,
        curobo_plan=True,
        enable_visualizer=False,
        # opt_viz_interval=10,
        viz_robot_mesh=False,
        enable_experiment_logging=False
    )

    tamp = TAMP(
        config=config,
        use_tetris_tuned_weights=None
    )

    tamp_server = TAMPServer(tamp)

    executor = MultiThreadedExecutor(num_threads=4)

    executor.add_node(tamp_server)

    try:
        tamp_server.get_logger().info("Starting TAMP server with MultiThreadedExecutor.")
        executor.spin()
    finally:
        executor.shutdown()
        tamp_server.destroy_node()
        rclpy.shutdown()