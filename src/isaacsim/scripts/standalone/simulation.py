import sys, os
import rclpy
from rclpy.node import Node
import numpy as np
from isaacsim import SimulationApp

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tamp_interfaces.srv import ChangeTool


ROBOT_STAGE_PATH = "/World/Robot"
ROOT_JOINT_PATH = ROBOT_STAGE_PATH + "/root_joint"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

CONFIG = {"renderer": "RaytracedLighting", "headless": False}


class Simulation(Node):
    
    def __init__(self):
        
        super().__init__("sdl_isaacsim")

        self.simulation_app = SimulationApp(CONFIG)

        import carb
        import omni.graph.core as og
        import usdrt.Sdf
        from isaacsim.core.api import World
        from isaacsim.core.utils import extensions, prims, rotations, stage, viewports
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Gf
        from isaacsim.core.utils.types import ArticulationAction

        extensions.enable_extension("isaacsim.ros2.sim_control")

        # Save Imports
        self.prims = prims
        self.ArticulationAction = ArticulationAction
        self.World = World

        self._saved_robot_joint_positions = None
        
        self.simulation_app.update()

        self.world = self.World(stage_units_in_meters=1.0)

        # Preparing stage
        viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

        from task import Task

        self.task = Task(name="task", robot_prim_path=ROBOT_STAGE_PATH, robot_name="fr5")
        self.world.add_task(self.task)
        self.world.reset()

        self.simulation_app.update()

        self.robot = self.world.scene.get_object("fr5")
        if self.robot is None:
            self.get_logger().error("Failed to get robot 'fr5' from scene after initial reset.")
            self.get_logger().error("This is likely due to the USD asset issue. Check startup warnings.")
            self.simulation_app.close()
            return
            
        self.robot.post_reset()

        self.simulation_app.update()

        # need to initialize physics getting any articulation etc.
        self.world.initialize_physics()

        self.arm_joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        self.arm_joint_ids = []

        if not self.update_joint_ids():
            self.get_logger().error("Failed to initialize joint IDs. Shutting down.")
            self.simulation_app.close()
            return

        self.world.play()

        self.timer_period = 1/60
        self.timer = self.create_timer(self.timer_period, self.step_cb)

        self.joint_states_pub = self.create_publisher(JointState, "isaac_joint_states", 10)
        self.arm_commands_sub = self.create_subscription(JointState, "isaac_arm_commands", self.arm_commands_cb, 10)
        self.gripper_commands_srv = self.create_service(SetBool, "isaac_gripper_commands", self.gripper_commands_cb)

        self.tool_change_srv = self.create_service(ChangeTool, "tool_change", self.tool_change_cb)

        self.get_logger().info("Simulation Start")

    
    def update_joint_ids(self) -> bool:

        self.get_logger().info("Updating joint IDs...")
        self.arm_joint_ids = []
        if self.robot is None:
            self.get_logger().error("Cannot update joint IDs, self.robot is None.")
            return False
        
        if not self.robot.is_valid():
             self.get_logger().error("Cannot update joint IDs, self.robot is not valid.")
             return False
            
        for joint_name in self.arm_joint_names:
            idx = self.robot.get_dof_index(joint_name)
            if idx == -1:
                self.get_logger().error(f"Failed to find joint '{joint_name}' on robot.")
                return False
            self.arm_joint_ids.append(idx)
        
        self.get_logger().info(f"Updated arm joint IDs: {self.arm_joint_ids}")
        return True


    def step_cb(self):

        if self.simulation_app.is_running():

            # step simulation
            self.world.step(render=True)
            
            if self.robot and self.robot.is_valid(): 
                js_msg = JointState()
                js_msg.header.stamp = self.get_clock().now().to_msg()
                js_msg.name = self.robot.dof_names
                js_msg.position = self.robot.get_joint_positions().tolist()
                js_msg.velocity = self.robot.get_joint_velocities().tolist()
                self.joint_states_pub.publish(js_msg)

        else:
            self.get_logger().info("Quit ROS2 Node")
            rclpy.try_shutdown()


    def arm_commands_cb(self, msg):
        
        if self.robot is None or not self.robot.is_valid():
            self.get_logger().warning("arm_commands_cb: Robot is not valid. Skipping command.")
            return

        action = self.ArticulationAction(joint_positions=msg.position, joint_indices=self.arm_joint_ids)
        self.robot.apply_action(action)


    def gripper_commands_cb(self, request, response):

        if self.robot is None or not self.robot.is_valid():
            self.get_logger().warning("gripper_commands_cb: Robot is not valid. Skipping command.")
            response.success = False
            response.message = "Robot is not valid (currently swapping?)"
            return response

        is_close = request.data

        if is_close:
            self.robot.gripper.close()
            response.message = "close gripper"
        else:
            self.robot.gripper.open()
            response.message = "open gripper"

        response.success = True
        
        return response
    

    def tool_change_cb(self, request, response):

        desired_tool = request.desired_tool
        
        try:

            observations = self.world.get_observations()
            self.task.current_positions = observations["current_positions"]
            self.task.current_orientations = observations["current_orientations"]

            self.task.desired_tool = desired_tool

            self._saved_robot_joint_positions = None
            if self.robot and self.robot.is_valid():
                self._saved_robot_joint_positions = self.robot.get_joint_positions(joint_indices=self.arm_joint_ids)
            
            self.timer.cancel()
            self.world.stop() 

            self.world.clear()

            self.simulation_app.update()
            
            self.world = self.World(stage_units_in_meters=1.0)
            self.world.add_task(self.task)

            self.world.reset()
            self.world.initialize_physics()

            self.simulation_app.update() 
            self.robot = self.world.scene.get_object("fr5")
            
            self.robot.post_reset()

            self.robot.set_joint_positions(
                positions=self._saved_robot_joint_positions,
                joint_indices=self.arm_joint_ids,
            )
            zero_vels = np.zeros(self.robot.num_dof, dtype="float32")
            self.robot.set_joint_velocities(zero_vels)

            self.update_joint_ids()
            
            self.world.play()
            self.timer = self.create_timer(self.timer_period, self.step_cb)
            self.get_logger().info("Physics simulation resumed. Tool change complete.")

            response.success = True
            response.message = "Robot/Tool change complete and simulation resumed."

        except ValueError as e:
            print(f"tool_change Service Error: {e}")

            response.success = False
            response.message = "Robot/Tool change Failed"


        return response
    


def main(args=None):

    rclpy.init(args=args)
    sim_node = None
    try:
        sim_node = Simulation()
        if sim_node and rclpy.ok() and sim_node.robot is not None:
            rclpy.spin(sim_node)
        elif sim_node:
             sim_node.get_logger().error("Simulation node initialized but robot not found. Shutting down.")
        else:
            print("Simulation node failed to initialize.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        print(f"ROS2 Spin Exception: {e}")
    finally:
        if sim_node:
            sim_node.get_logger().info("Shutting down simulation...")
            if sim_node.world:
                sim_node.world.stop()
            sim_node.simulation_app.close()
            sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()