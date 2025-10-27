#!/usr/bin/env python3

from cutamp.config import TAMPConfiguration

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration as RclpyDuration # 이름 충돌을 피하기 위해 별칭 사용

from tamp_interfaces.msg import PlanStep
from tamp_interfaces.srv import Plan, Execute, SetTampEnv
from builtin_interfaces.msg import Duration
from simulation_interfaces.msg import EntityState
from simulation_interfaces.srv import GetEntityState
from geometry_msgs.msg import Pose

from tamp import TAMP
from tamp_env import load_demo_env

from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


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
        # 모든 서비스 서버에 동일한 reentrant_group을 할당합니다.
        self.plan_srv = self.create_service(
            Plan, 'tamp_plan', self.tamp_plan_cb, 
            callback_group=self.reentrant_group  # <--- 이 부분을 추가하세요
        )
        self.execute_srv = self.create_service(
            Execute, 'plan_execute', self.execute_plan_cb, 
            callback_group=self.reentrant_group  # <--- 이 부분을 추가하세요
        )
        self.set_tamp_env_srv = self.create_service(
            SetTampEnv, 'set_tamp_env', self.set_tamp_env_cb, 
            callback_group=self.reentrant_group  # <--- 이 부분을 추가하세요
        )

        # clients
        self.get_entity_state_cli = self.create_client(
            GetEntityState, 'get_entity_state', 
            callback_group=self.reentrant_group
        )

        # robot states
        self.joint_states = JointState()
        self.joint_commands = JointState()

        
    # set_tamp_env_cb를 async 함수로 변경
    async def set_tamp_env_cb(self, request, response):
        get_entity_state_request = GetEntityState.Request()
        get_entity_state_request.entity = request.entity

        while not self.get_entity_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # 비동기 서비스 호출
        try:
            get_entity_state_response = await self.get_entity_state_cli.call_async(get_entity_state_request)
            
            # tamp에 정의된 movables의 순서는 미리 정해놓기
            if get_entity_state_response.result.result == 1:
                entity_pose = []
                entity_position = get_entity_state_response.state.pose.position
                entity_orientation = get_entity_state_response.state.pose.orientation
                entity_pose.append(entity_position.x)
                entity_pose.append(entity_position.y)
                entity_pose.append(0.08)
                # entity_pose.append(entity_position.z)
                entity_pose.append(entity_orientation.w)
                entity_pose.append(entity_orientation.x)
                entity_pose.append(entity_orientation.y)
                entity_pose.append(entity_orientation.z)

                self.tamp.update_env(ids=[0], poses=[entity_pose]) # TODO: ids는 바꿔줘야 함.
                response.success = True
                self.get_logger().info(f"Service call successful")
            else:
                response.success = False
                self.get_logger().error(f"get_entity_state error: {get_entity_state_response.result.result}")
        
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")
            response.success = False

        return response


    def joint_states_cb(self, msg):
        self.joint_states = msg

    def tamp_plan_cb(self, request, response):

        self.get_logger().info("TAMP plan start")
        q_init = self.joint_states.position[:6].tolist() # num_dof = 6

        curobo_plan, total_num_satisfying = self.tamp.plan(q_init, None)

        self.plan_to_execute = curobo_plan
        
        encoded_curobo_plan = self.process_plan(curobo_plan)

        response.plan_success = (total_num_satisfying > 0)
        response.curobo_plan = encoded_curobo_plan
        response.total_num_satisfying = total_num_satisfying

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
                self.get_logger().info("Wait finished. Proceeding to the next step.")
            else:
                return

        if not self.plan_to_execute or self.plan_index >= len(self.plan_to_execute):
            self.get_logger().info("Plan execution finished.")
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
                self.get_logger().info(f"Trajectory part (plan_index: {self.plan_index}) finished.")
                step_finished = True
                self.current_plan_step = 0 # 다음 궤적을 위해 초기화

        elif plan_type == "gripper":
            self.execute_gripper_action(current_plan_part)
            self.get_logger().info(f"Gripper action (plan_index: {self.plan_index}) finished.")
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
                self.get_logger().info(f"Waiting for {self.wait_duration.nanoseconds / 1e9} seconds after step completion.")


    def execute_gripper_action(self, plan_part):
        self.joint_commands.header.stamp = self.get_clock().now().to_msg()
        self.joint_commands.name = ["gripper_finger1_joint"]

        if plan_part["action"] == "close":
            self.joint_commands.position = [0.6524]
        else: # open
            self.joint_commands.position = [0.0] # Isaac Sim Panda 그리퍼의 일반적인 최대 개방 값

        self.get_logger().info(f"Executing gripper action: {plan_part['action']}")
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
        num_initial_plans=30,
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

    executor = MultiThreadedExecutor()

    executor.add_node(tamp_server)

    try:
        tamp_server.get_logger().info("Starting TAMP server with MultiThreadedExecutor.")
        executor.spin()
    finally:
        executor.shutdown()
        tamp_server.destroy_node()
        rclpy.shutdown()