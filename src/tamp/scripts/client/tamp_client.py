#!/usr/bin/env python3
import cmd
import sys

import rclpy
from rclpy.client import Client
from tamp_interfaces.srv import (
    Plan, Execute, SetTampEnv, SetTampCfg, ToolChange, MoveToTarget, MoveToTargetJs, GetRobotInfo, GetToolInfo
)
from std_srvs.srv import SetBool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ControlSuiteShell(cmd.Cmd):

    intro = (
        bcolors.OKBLUE
        + "Welcome to the control suite shell.\nType help or ? to list commands.\n"
        + bcolors.ENDC
    )
    prompt = "(csuite) "

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node(
            "cutamp_client",
            parameter_overrides=[
                rclpy.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True)
            ]
        )

        # Service Clients
        self.plan_client = self.node.create_client(Plan, 'tamp_plan')
        self.execute_client = self.node.create_client(Execute, 'plan_execute')
        self.set_tamp_env_client = self.node.create_client(SetTampEnv, 'set_tamp_env')
        self.set_tamp_cfg_client = self.node.create_client(SetTampCfg, "set_tamp_cfg")
        self.tool_change_client = self.node.create_client(ToolChange, 'tool_change')
        self.gripper_commands_client = self.node.create_client(SetBool, "isaac_gripper_commands")
        self.move_to_target_client = self.node.create_client(MoveToTarget, "move_to_target")
        self.move_to_target_js_client = self.node.create_client(MoveToTargetJs, "move_to_target_js")
        self.get_robot_info_client = self.node.create_client(GetRobotInfo, "get_robot_info")
        self.get_tool_info_client = self.node.create_client(GetToolInfo, "get_tool_info")

        while (
            not self.plan_client.wait_for_service(timeout_sec=1.0) and 
            not self.execute_client.wait_for_service(timeout_sec=1.0) and
            not self.set_tamp_env_client.wait_for_service(timeout_sec=1.0) and
            not self.set_tamp_cfg_client.wait_for_service(timeout_sec=1.0) and 
            not self.tool_change_client.wait_for_service(timeout_sec=1.0) and 
            not self.gripper_commands_client.wait_for_service(timeout_sec=1.0) and
            not self.move_to_target_client.wait_for_service(timeout_sec=1.0) and 
            not self.get_tool_info_client.wait_for_service(timeout_sec=1.0) and
            not self.get_robot_info_client.wait_for_service(timeout_sec=1.0) and 
            not self.get_tool_info_client.wait_for_service(timeout_sec=1.0) 
        ):
            self.get_logger().info('service not available, waiting again...')

        self.node.get_logger().info('All action and service servers are ready.')


    def do_plan(self, arg):
        env_name = arg.strip() if arg else "pouring" # Default to "pouring" if no arg
        if not env_name:
            self.node.get_logger().info("Usage: plan <env_name>")
            return
        request = Plan.Request()
        request.env_name = env_name
        # TODO: We have to add another term that is related to SDL parserd pddlstream order
        response = self._call_service_and_wait(self.plan_client, request)

        if response:
            self.node.get_logger().info(f"Service call successful, result: {response.plan_success}")
        else:
            self.node.get_logger().warn("Service call failed")


    def do_execute(self, arg):

        request = Execute.Request()

        response = self._call_service_and_wait(self.execute_client, request)

        if response:
            self.node.get_logger().info(f"Service call successful, result: {response.execute_success}")
        else:
            self.node.get_logger().warn("Service call failed")


    def do_set_tamp_env(self, arg):

        request = SetTampEnv.Request()

        if arg.strip() == "pouring":
            request.env_name = "pouring"
            request.entities = ["beaker", "flask", "magnet"]
            request.movables = ["beaker", "flask"]
            request.statics = ["table", "goal_region", "stirrer", "magnet"]
            request.ex_collision = ["pour_region"]
        elif arg.strip() == "stirring":
            request.env_name = "stirring"
            request.entities = ["beaker", "flask", "magnet"]
            request.movables = ["flask", "magnet"]
            request.statics = ["table", "stirrer", "beaker", "goal_region"]
            request.ex_collision = ["beaker_region"]
        else:
            raise ValueError("arg must be 'pouring' or 'stirring'")

        response = self._call_service_and_wait(self.set_tamp_env_client, request)
        if response:
            self.node.get_logger().info(f"Service call successful, result: {response.success}")
        else:
            self.node.get_logger().warn("Service call failed")

    def do_set_tamp_cfg(self, arg):

        request = SetTampCfg.Request()

        desired_tool = arg.strip().lower()
        assert desired_tool in {"empty", "ag95", "vgc10", "dh3"}

        request.curobo_plan = True
        request.enable_visualizer = False
        request.viz_robot_mesh = False
        request.enable_experiment_logging = False

        robot_name = "fr5"

        if arg.strip() == "empty":
            request.robot = robot_name
        else:
            request.robot = robot_name + "_" + desired_tool

        response = self._call_service_and_wait(self.set_tamp_cfg_client, request)
        if response:
            self.node.get_logger().info(f"Service call successful, result: {response.success}")
        else:
            self.node.get_logger().warn("Service call failed")

        

    
    def do_tool_change(self, arg):

        # get robot info
        get_robot_info_request = GetRobotInfo.Request()
        get_robot_info_response = self._call_service_and_wait(self.get_robot_info_client, get_robot_info_request)
        q_init = get_robot_info_response.q_init
        joint_names = get_robot_info_response.joint_names
        current_tool = get_robot_info_response.current_tool
        desired_tool = arg.strip().lower()

        assert current_tool in ["empty", "ag95", "vgc10", "dh3"], f"Error: Tool '{arg.strip()}' is not supported."
        assert desired_tool in ["ag95", "vgc10", "dh3"], f"Error: Tool '{arg.strip()}' is not supported."
        
        # get tool info
        get_tool_info_request = GetToolInfo.Request()
        get_tool_info_request.current_tool = current_tool
        get_tool_info_request.desired_tool = desired_tool
        get_tool_info_response = self._call_service_and_wait(self.get_tool_info_client, get_tool_info_request)
        current_tool_position = get_tool_info_response.current_tool_position
        current_tool_orientation = get_tool_info_response.current_tool_orientation
        desired_tool_position = get_tool_info_response.desired_tool_position
        desired_tool_orientation = get_tool_info_response.desired_tool_orientation

        if current_tool == "empty":
            # directly move to desired tool pose
            move_to_target_request = MoveToTarget.Request()
            move_to_target_request.q_init = q_init
            move_to_target_request.target_position = desired_tool_position
            move_to_target_request.target_orientation = desired_tool_orientation

            self.do_set_tamp_env(arg="pouring") # For Update Env

            move_to_target_response = self._call_service_and_wait(self.move_to_target_client, move_to_target_request)
            time.sleep(3.0)

            # grip tool
            tool_change_request = ToolChange.Request()
            tool_change_request.desired_tool = desired_tool

            tool_change_response = self._call_service_and_wait(self.tool_change_client, tool_change_request)
            self.do_set_tamp_cfg(arg=desired_tool) # Change Robot Cfg
        else:
            # move to home qpos -> move to current tool pose
            # TODO: Visual Grippers are recognized as colliders
            # move to home position
            home_pos = [0.0, -1.05, -2.18, -1.57, 1.57, 0.0]
            move_to_target_js_request = MoveToTargetJs.Request()
            move_to_target_js_request.q_init = q_init
            move_to_target_js_request.q_des = home_pos
            self.do_set_tamp_env(arg="pouring")

            move_to_target_js_response = self._call_service_and_wait(self.move_to_target_js_client, move_to_target_js_request)
            time.sleep(3.0)
            
            # move to current tool pose
            self.do_set_tamp_cfg(arg="empty") # Change Robot Cfg
            move_to_target_request = MoveToTarget.Request()
            move_to_target_request.q_init = q_init
            move_to_target_request.target_position = current_tool_position
            move_to_target_request.target_orientation = current_tool_orientation

            self.do_set_tamp_env(arg="pouring")

            move_to_target_response = self._call_service_and_wait(self.move_to_target_client, move_to_target_request)
            time.sleep(3.0)

            # release tool
            tool_change_request = ToolChange.Request()
            tool_change_request.desired_tool = "empty"

            tool_change_response = self._call_service_and_wait(self.tool_change_client, tool_change_request)
            self.do_set_tamp_cfg(arg="empty") # Change Robot Cfg

            # move to desired_tool pose
            get_robot_info_request = GetRobotInfo.Request()
            get_robot_info_response = self._call_service_and_wait(self.get_robot_info_client, get_robot_info_request)
            q_init = get_robot_info_response.q_init

            move_to_target_request = MoveToTarget.Request()
            move_to_target_request.q_init = q_init
            move_to_target_request.target_position = desired_tool_position
            move_to_target_request.target_orientation = desired_tool_orientation

            self.do_set_tamp_env(arg="pouring")

            move_to_target_response = self._call_service_and_wait(self.move_to_target_client, move_to_target_request)
            time.sleep(3.0)

            # grip tool
            tool_change_request = ToolChange.Request()
            tool_change_request.desired_tool = desired_tool

            tool_change_response = self._call_service_and_wait(self.tool_change_client, tool_change_request)
            self.do_set_tamp_cfg(arg=desired_tool) # Change Robot Cfg


        if move_to_target_response and tool_change_response:
            self.node.get_logger().info(f"Service call successful, result: {move_to_target_response.success}")
        else:
            self.node.get_logger().warn("Service call failed")

    
    def do_grasp(self, arg):

        request = SetBool.Request()

        if arg.strip().lower() == "close":
            request.data = True
        elif arg.strip().lower() == "open":
            request.data = False
        else:
            raise ValueError("arg is must be 'close' or 'open'")
        
        response = self._call_service_and_wait(self.gripper_commands_client, request)
        if response:
            self.node.get_logger().info(f"Service call successful, result: {response.success}")
        else:
            self.node.get_logger().warn("Service call failed")
            


    ##############################################################################
    def _call_service_and_wait(self, client: Client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is not None:
            return response
        else:
            self.node.get_logger().error(f"Exception while calling service: {future.exception()}")
            return None


    def do_quit(self, arg):
        """Quit shell"""
        self.node.get_logger().info("Shutting down ROS 2 …")
        self.node.destroy_node()
        rclpy.shutdown()
        return True
    

    def do_EOF(self, arg):
        return self.do_quit(arg)




if __name__ == "__main__":
    try:
        ControlSuiteShell().cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupt - shutting down …")
        rclpy.shutdown()
        sys.exit(0)