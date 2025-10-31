#!/usr/bin/env python3
import cmd
import sys

import rclpy
from rclpy.client import Client
from tamp_interfaces.srv import Plan, Execute, SetTampEnv


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

        while (
            not self.plan_client.wait_for_service(timeout_sec=1.0) and 
            not self.execute_client.wait_for_service(timeout_sec=1.0) and
            not self.set_tamp_env_client.wait_for_service(timeout_sec=1.0)
        ):
            self.get_logger().info('service not available, waiting again...')

        self.node.get_logger().info('All action and service servers are ready.')


    def do_plan(self, arg):
        env_name = arg.strip() if arg else "pouring" # Default to "pouring" if no arg
        if not env_name:
            print("Usage: plan <env_name>")
            return
        request = Plan.Request()
        request.env_name = env_name
        # TODO: We have to add another term that is related to SDL parserd pddlstream order
        response = self._call_service_and_wait(self.plan_client, request)

        if response:
            print(f"Service call successful, result: {response.plan_success}")
        else:
            print("Service call failed")


    def do_execute(self, arg):

        request = Execute.Request()

        response = self._call_service_and_wait(self.execute_client, request)

        if response:
            print(f"Service call successful, result: {response.execute_success}")
        else:
            print("Service call failed")


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
            print(f"Service call successful, result: {response.success}")
        else:
            print("Service call failed")


    ##############################################################################
    def _call_service_and_wait(self, client: Client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is not None:
            return response
        else:
            print(f"Exception while calling service: {future.exception()}")
            return None

    def do_quit(self, arg):
        """Quit shell"""
        print("Shutting down ROS 2 …")
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