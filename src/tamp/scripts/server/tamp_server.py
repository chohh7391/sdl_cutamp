#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration as RclpyDuration # 이름 충돌을 피하기 위해 별칭 사용

from tamp_interfaces.msg import PlanStep
from tamp_interfaces.srv import Plan, Execute, SetTampEnv
from builtin_interfaces.msg import Duration
from simulation_interfaces.srv import GetEntityState

from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from typing import Optional, List, Dict

from cutamp.algorithm import run_cutamp
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
from cutamp.envs import TAMPEnvironment

from envs.utils import load_demo_env, TAMPEnvManager


class TAMP:

    def __init__(
        self,
        config: TAMPConfiguration,
        env: TAMPEnvironment,
        use_tetris_tuned_weights: bool = None,
    ):
        validate_tamp_config(config)

        self.env_manager = TAMPEnvManager()

        self.config = config
        self.env = env

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


    def plan(
        self,
        q_init: Optional[List[float]] = None,
        experiment_id: Optional[str] = None
    ):
        self.curobo_plan, self.total_num_satisfying = run_cutamp(
            env=self.env,
            config=self.config,
            cost_reducer=self.cost_reducer,
            constraint_checker=self.constraint_checker,
            q_init=q_init,
            experiment_id=experiment_id
        )

        return self.curobo_plan, self.total_num_satisfying


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
        self.execution_timer = None

        # duration between plan_steps 
        self.is_waiting_after_step = False
        self.wait_start_time = None
        self.wait_duration = RclpyDuration(seconds=0.4)

        # publisher
        self.joint_command_publisher = self.create_publisher(JointState, "isaac_joint_commands", 10)

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

        # clients
        self.get_entity_state_cli = self.create_client(
            GetEntityState, 'get_entity_state', 
            callback_group=self.reentrant_group
        )

        # robot states
        self.joint_states = JointState()
        self.joint_commands = JointState()


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
            
            if self.execution_timer is not None:
                self.execution_timer.cancel()

            self.plan_index = 0
            self.current_plan_step = 0
            self.is_waiting_after_step = False # 실행 시작 시 대기 상태 초기화
            
            timer_period = 0.016 # 기본 타이머 주기
            for plan_part in self.plan_to_execute:
                if plan_part['type'] == 'trajectory':
                    timer_period = plan_part['dt']
                    break
            
            self.execution_timer = self.create_timer(timer_period, self.timer_callback)
            response.execute_success = True
        else:
            self.get_logger().warn("No plan to execute.")
            response.execute_success = False


        return response


    def timer_callback(self):
        if self.is_waiting_after_step:
            if self.get_clock().now() - self.wait_start_time >= self.wait_duration:
                self.is_waiting_after_step = False
            else:
                return

        if not self.plan_to_execute or self.plan_index >= len(self.plan_to_execute):
            if self.execution_timer is not None:
                self.execution_timer.cancel()
                self.execution_timer = None
            # self.plan_to_execute = None
            return

        current_plan_part = self.plan_to_execute[self.plan_index]
        plan_type = current_plan_part.get("type")
        
        step_finished = False

        if plan_type == "trajectory":
            plan_trajectory = current_plan_part["plan"]
            num_waypoints = plan_trajectory.position.shape[0]

            if self.current_plan_step < num_waypoints:
                self.joint_commands.header.stamp = self.get_clock().now().to_msg()
                self.joint_commands.name = plan_trajectory.joint_names
                self.joint_commands.position = plan_trajectory.position[self.current_plan_step].tolist()
                self.joint_commands.velocity = plan_trajectory.velocity[self.current_plan_step].tolist()
                self.joint_commands.effort = plan_trajectory.acceleration[self.current_plan_step].tolist()
                
                self.joint_command_publisher.publish(self.joint_commands)
                
                self.current_plan_step += 1
            
            if self.current_plan_step >= num_waypoints:
                step_finished = True
                self.current_plan_step = 0 # 다음 궤적을 위해 초기화

        elif plan_type == "gripper":
            self.execute_gripper_action(current_plan_part)
            step_finished = True
            
        else:
            self.get_logger().error(f"Unknown plan type: {plan_type}")
            if self.execution_timer is not None:
                self.execution_timer.cancel()
            return

        if step_finished:
            self.plan_index += 1
            
            if self.plan_index < len(self.plan_to_execute):
                self.is_waiting_after_step = True
                self.wait_start_time = self.get_clock().now()


    def execute_gripper_action(self, plan_part):
        self.joint_commands.header.stamp = self.get_clock().now().to_msg()
        self.joint_commands.name = ["gripper_finger1_joint"]

        if plan_part["action"] == "close":
            self.joint_commands.position = [0.4] # 0.6524 is maximum value. But this value occurs penatration
        else: # open
            self.joint_commands.position = [0.0]

        self.joint_command_publisher.publish(self.joint_commands)

    def _call_service_and_wait(self, client: Client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            return response
        else:
            print(f"Exception while calling service: {future.exception()}")
            return None



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

    env = load_demo_env(name="pouring")

    tamp = TAMP(
        config=config,
        env=env,
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