from abc import ABC, abstractmethod
from typing import Optional, Dict

import numpy as np
import omni
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleXFormPrim, SingleRigidPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units, add_reference_to_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera, SingleViewDepthSensor
from isaacsim.robot.manipulators.grippers import Gripper, ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from pxr import Gf
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api.robots.robot import Robot
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from typing import List, Optional

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.api.objects import DynamicCuboid
from fr5 import FR5
import os

class Task(ABC, BaseTask):

    def __init__(self, name: str, robot_prim_path: str, robot_name: str) -> None:
        
        BaseTask.__init__(self, name=name)
        self._asset_root_path = get_assets_root_path()
        if self._asset_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")
        self._robot_prim_path = robot_prim_path
        self._robot_name = robot_name

        self.current_positions = None
        self.current_orientations = None
        self.desired_tool = None

        self.default_positions = {
            "table": np.array([0.0, 0.0, -0.01]),
            "stirrer": np.array([0.3, 0.3, 0.015]),
            "beaker": np.array([0.4, 0.2, 0.06]),
            "flask": np.array([0.0, 0.5, 0.04]),
            "magnet": np.array([-0.4, 0.3, 0.015]),
        }
        self.default_orientations = {
            "table": np.array([1.0, 0.0, 0.0, 0.0]),
            "stirrer": np.array([1.0, 0.0, 0.0, 0.0]),
            "beaker": np.array([1.0, 0.0, 0.0, 0.0]),
            "flask": np.array([1.0, 0.0, 0.0, 0.0]),
            "magnet": np.array([1.0, 0.0, 0.0, 0.0]),
        }

        return
    
    def set_up_scene(self, scene: Scene) -> None:
        super().set_up_scene(scene)

        scene.add_default_ground_plane(z_position=-0.1)

        self.set_robot(self.desired_tool)
        self.set_object(self.current_positions, self.current_orientations)


    def set_robot(self, desired_tool = None) -> FR5:

        if desired_tool is None:
            desired_tool = "ag95"
        else:
            desired_tool = desired_tool.lower()

        if desired_tool == "ag95":
            robot_asset_path = os.path.join(
                os.path.abspath(__file__),
                "..", "..", "..", "..",
                "tamp/content/assets/robot/dcp_description/usd/fr5_ag95/fr5_ag95.usd"
            )
            robot_prim_path = find_unique_string_name(
                initial_name=self._robot_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            robot_name = find_unique_string_name(
                initial_name=self._robot_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
            )

            self._robot = FR5(
                prim_path=robot_prim_path,
                name=robot_name,
                usd_path=robot_asset_path,
                end_effector_prim_name="gripper_finger2_finger_tip_link",
                gripper_dof_names=["gripper_finger1_joint"],
                use_mimic_joints=True,
                gripper_open_position=np.array([0.0]),
                gripper_closed_position=np.array([0.6524]),
            )
            self._robot.joints_default_state = np.array([
                0.0, -1.05, -2.18, -1.57, 1.57, 0.0, # Arm joint position
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Gripper joint position
            ])
            
        elif desired_tool == "2f_85":
            robot_asset_path = os.path.join(os.path.abspath(__file__), "..", "..", "..", "..", "tamp/content/assets/robot/dcp_description/usd/fr5_2f_85/fr5_2f_85.usd")
            robot_prim_path = find_unique_string_name(
                initial_name=self._robot_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            robot_name = find_unique_string_name(
                initial_name=self._robot_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
            )

            self._robot = FR5(
                prim_path=robot_prim_path,
                name=robot_name,
                usd_path=robot_asset_path,
                end_effector_prim_name="Robotiq_2F_85_edit/Robotiq_2F_85/right_inner_finger",
                gripper_dof_names=["finger_joint"],
                use_mimic_joints=True,
                gripper_open_position=np.array([0.0]),
                gripper_closed_position=np.array([0.82]),
            )
            self._robot.joints_default_state = np.array([
                0.0, -1.05, -2.18, -1.57, 1.57, 0.0, # Arm joint position
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Gripper joint position
            ])

        elif desired_tool == "vgc10":
            robot_asset_path = os.path.join(
                os.path.abspath(__file__),
                "..", "..", "..", "..",
                "tamp/content/assets/robot/dcp_description/usd/fr5_vgc10/fr5_vgc10.usd"
            )
            robot_prim_path = find_unique_string_name(
                initial_name=self._robot_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            robot_name = find_unique_string_name(
                initial_name=self._robot_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
            )

            self._robot = FR5(
                prim_path=robot_prim_path,
                name=robot_name,
                usd_path=robot_asset_path,
                end_effector_prim_name="wrist3_link",
                is_surface_gripper=True,
                surface_gripper_path=robot_prim_path + "/surface_gripper",
            )

        else:
            raise ValueError("Available Grippers are only 'ag95', '2f_85', 'vgc10'")


        self.scene.add(self._robot)

        return self._robot
    
    def set_object(self, current_positions = None, current_orientations = None) -> FR5:

        if current_positions is None:
            current_positions = self.default_positions
            current_orientations = self.default_orientations


        self.table = self.scene.add(
            FixedCuboid(
                prim_path="/World/table",
                name="table",
                position=current_positions["table"],
                orientation=current_orientations["table"],
                scale=np.array([1.5, 1.5, 0.02]),
                size=1.0,
                color=np.array([0.922, 0.769, 0.569])
            )
        )
        self.stirrer = self.scene.add(
            DynamicCuboid(
                prim_path="/World/stirrer",
                name="stirrer",
                position=current_positions["stirrer"],
                orientation=current_orientations["stirrer"],
                scale=np.array([0.1, 0.1, 0.03]),
                size=1.0,
                color=np.array([0.922, 0.769, 0.569])
            )
        )
        self.beaker = self.scene.add(
            DynamicCuboid(
                prim_path="/World/beaker",
                name="beaker",
                position=current_positions["beaker"],
                orientation=current_orientations["beaker"],
                scale=np.array([0.05, 0.05, 0.12]),
                size=1.0,
                color=np.array([0.0, 0.0, 1.0])
            )
        )
        self.flask = self.scene.add(
            DynamicCuboid(
                prim_path="/World/flask",
                name="flask",
                position=current_positions["flask"],
                orientation=current_orientations["flask"],
                scale=np.array([0.06, 0.06, 0.08]),
                size=1.0,
                color=np.array([0.0, 1.0, 0.0])
            )
        )
        self.magnet = self.scene.add(
            DynamicCuboid(
                prim_path="/World/magnet",
                name="magnet",
                position=current_positions["magnet"],
                orientation=current_orientations["magnet"],
                scale=np.array([0.045, 0.045, 0.03]),
                size=1.0,
                color=np.array([1.0, 0.0, 1.0])
            )
        )

        

    def get_observations(self) -> Dict:
        
        table_pos, table_ori = self.table.get_world_pose()
        stirrer_pos, stirrer_ori = self.stirrer.get_world_pose()
        beaker_pos, beaker_ori = self.beaker.get_world_pose()
        flask_pos, flask_ori = self.flask.get_world_pose()
        magnet_pos, magnet_ori = self.magnet.get_world_pose()
        
        observations = {
            "current_positions": {
                "table": table_pos,
                "stirrer": stirrer_pos,
                "beaker": beaker_pos,
                "flask": flask_pos,
                "magnet": magnet_pos,
            },
            "current_orientations": {
                "table": table_ori,
                "stirrer": stirrer_ori,
                "beaker": beaker_ori,
                "flask": flask_ori,
                "magnet": magnet_ori,
            }
        }

        return observations