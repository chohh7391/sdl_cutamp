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

        self._cube_initial_positions = np.array([
            [0.5, 0.45, 0.45],
            [0.3, -0.5, 0.45],
            [0.0, 0.45, 0.45],
        ])

        return
    
    def set_up_scene(self, scene: Scene) -> None:
        super().set_up_scene(scene)

        scene.add_default_ground_plane()

        self._robot = self.set_robot()
        self.set_object()

        self.scene.add(self._robot)

    def set_robot(self) -> FR5:
        robot_asset_path =  "/home/home/sdl_ws/src/sdl/content/assets/robot/dcp_description/usd/fr5_ag95/fr5_ag95.usd"
        robot_prim_path = find_unique_string_name(
            initial_name=self._robot_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        robot_name = find_unique_string_name(
            initial_name=self._robot_name, is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        return FR5(
            prim_path=robot_prim_path,
            name=robot_name,
            usd_path=robot_asset_path,
            end_effector_prim_name="gripper_finger2_finger_tip_link",
            gripper_dof_names=["gripper_finger1_joint"],
            use_mimic_joints=True,
            gripper_open_position=np.array([0.0]),
            gripper_closed_position=np.array([0.6524]),
        )
    
    def set_object(self) -> FR5:

        for i in range(3):
            color = np.random.uniform(size=(3,))
            cube_prim_path = find_unique_string_name(
                initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            cube_name = find_unique_string_name(
                initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
            self.scene.add(
                DynamicCuboid(
                    name=cube_name,
                    position=self._cube_initial_positions[i],
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                    prim_path=cube_prim_path,
                    scale=np.array([0.0515, 0.0515, 0.09]),
                    size=1.0,
                    color=color,
                )
            )

    def get_observations(self) -> Dict:

        pass