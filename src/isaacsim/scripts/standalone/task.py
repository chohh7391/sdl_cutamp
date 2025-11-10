from abc import ABC
from typing import Dict

import numpy as np
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.storage.native import get_assets_root_path

import numpy as np
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
        self.current_tool = None

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

        self.set_object(self.current_positions, self.current_orientations)
        self.set_robot(self.desired_tool)
        


    def set_robot(self, desired_tool = None) -> FR5:

        if desired_tool is None:
            desired_tool = "empty"
        else:
            desired_tool = desired_tool.lower()

        if desired_tool == "empty":

            robot_asset_path = os.path.join(
                os.path.abspath(__file__),
                "..", "..", "..", "..",
                "tamp/content/assets/robot/dcp_description/usd/fr5/fr5.usd"
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
            )
            self._robot.joints_default_state = np.array([
                0.0, -1.05, -2.18, -1.57, 1.57, 0.0, # Arm joint position
            ])

            self.gripper_ag95.set_visibility(True)
            self.gripper_vgc10.set_visibility(True)
            self.gripper_dh3.set_visibility(True)

        elif desired_tool == "ag95":
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
            self.gripper_ag95.set_visibility(False)
            
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
                end_effector_prim_name="suction",
                is_surface_gripper=True,
                surface_gripper_path=robot_prim_path + "/SurfaceGripper",
            )

            self._robot.joints_default_state = np.array([
                0.0, -1.05, -2.18, -1.57, 1.57, 0.0, # Arm joint position
            ])
            self.gripper_vgc10.set_visibility(False)

        elif desired_tool == "dh3":
            robot_asset_path = os.path.join(
                os.path.abspath(__file__),
                "..", "..", "..", "..",
                "tamp/content/assets/robot/dcp_description/usd/fr5_dh3/fr5_dh3.usd"
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
                end_effector_prim_name="finger3_tip_link",
                gripper_dof_names=["finger1_joint"],
                use_mimic_joints=True,
                gripper_open_position=np.array([0.0]),
                gripper_closed_position=np.array([1.16]),
            )
            self._robot.joints_default_state = np.array([
                0.0, -1.05, -2.18, -1.57, 1.57, 0.0, # Arm joint position
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Gripper joint position
            ])
            self.gripper_dh3.set_visibility(False)

        else:
            raise ValueError("Available Grippers are only 'empty', 'ag95', 'vgc10', 'dh3'")

        self.current_tool = desired_tool
        
        self.scene.add(self._robot)

        return self._robot
    
    def set_object(self, current_positions = None, current_orientations = None) -> FR5:

        if current_positions is None:
            current_positions = self.default_positions
            current_orientations = self.default_orientations


        # Objects
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
            FixedCuboid(
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

        gripper_visual_asset_path = os.path.join(
            os.path.abspath(__file__),
            "..", "..", "..", "..",
            "tamp/content/assets/robot/dcp_description/usd/gripper_visual"
        )

        # ag95
        add_reference_to_stage(
            usd_path=os.path.join(gripper_visual_asset_path, "ag95", "ag95.usd"),
            prim_path="/World/gripper_visual/gripper_ag95"
        )
        self.gripper_ag95 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_ag95",
            name="gripper_ag95"
        )
        self.gripper_ag95.set_world_pose(
            position=[-0.6, -0.4, 0.25],
            orientation=[0, 0, 1, 0],
        )

        # vgc10
        add_reference_to_stage(
            usd_path=os.path.join(gripper_visual_asset_path, "vgc10", "vgc10.usd"),
            prim_path="/World/gripper_visual/gripper_vgc10"
        )
        self.gripper_vgc10 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_vgc10",
            name="gripper_vgc10"
        )
        self.gripper_vgc10.set_world_pose(
            position=[-0.6, 0.0, 0.25],
            orientation=[0, 0, 1, 0],
        )

        # dh3
        add_reference_to_stage(
            usd_path=os.path.join(gripper_visual_asset_path, "dh3", "dh3.usd"),
            prim_path="/World/gripper_visual/gripper_dh3"
        )
        self.gripper_dh3 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_dh3",
            name="gripper_dh3"
        )
        self.gripper_dh3.set_world_pose(
            position=[-0.6, 0.4, 0.25],
            orientation=[0, 0, 1, 0],
        )

        # gripper_base_link xform
        self.gripper_base_ag95 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_ag95/gripper_base_link",
            name="gripper_base_ag95"
        )
        self.gripper_base_vgc10 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_vgc10/gripper_base_link",
            name="gripper_base_vgc10"
        )
        self.gripper_base_dh3 = SingleXFormPrim(
            prim_path="/World/gripper_visual/gripper_dh3/gripper_base_link",
            name="gripper_base_dh3"
        )



    def get_observations(self) -> Dict:
        
        # object pose
        table_pos, table_ori = self.table.get_world_pose()
        stirrer_pos, stirrer_ori = self.stirrer.get_world_pose()
        beaker_pos, beaker_ori = self.beaker.get_world_pose()
        flask_pos, flask_ori = self.flask.get_world_pose()
        magnet_pos, magnet_ori = self.magnet.get_world_pose()

        # gripper base pose
        gripper_base_ag95_pos, gripper_base_ag95_ori = self.gripper_base_ag95.get_world_pose()
        gripper_base_vgc10_pos, gripper_base_vgc10_ori = self.gripper_base_vgc10.get_world_pose()
        gripper_base_dh3_pos, gripper_base_dh3_ori = self.gripper_base_dh3.get_world_pose()
        
        # observation dict
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
            },
            "gripper_base_position": {
                "empty": [0.0, 0.0, 0.0],
                "ag95": gripper_base_ag95_pos.tolist(),
                "vgc10": gripper_base_vgc10_pos.tolist(),
                "dh3": gripper_base_dh3_pos.tolist(),
            },
            "gripper_base_orientation": {
                "empty": [1.0, 0.0, 0.0, 0.0],
                "ag95": gripper_base_ag95_ori.tolist(),
                "vgc10": gripper_base_vgc10_ori.tolist(),
                "dh3": gripper_base_dh3_ori.tolist(),
            },
        }

        return observations