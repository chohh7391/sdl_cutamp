from typing import Optional
import torch
from curobo.geom.types import Cuboid, Cylinder
from curobo.types.base import TensorDeviceType
from cutamp.envs import TAMPEnvironment
from cutamp.envs.utils import unit_quat
from cutamp.tamp_domain import HandEmpty, On, Poured
from cutamp.utils.shapes import MultiSphere

from envs.utils import ENTITIES

def default_env(
    tensor_args: TensorDeviceType = TensorDeviceType(),
) -> TAMPEnvironment:
    
    env = TAMPEnvironment(
        name="default_env",
        movables=[
            ENTITIES["beaker"],
            ENTITIES["flask"],
            ENTITIES["magnet"],
            # ENTITIES["obstacle_1"],
            # ENTITIES["obstacle_2"],
        ],
        statics=[
            ENTITIES["table"],
            ENTITIES["stirrer"],
        ],
        ex_collision=[
            ENTITIES["goal_region"],
            ENTITIES["pour_region"],
            ENTITIES["beaker_region"],
        ],
        type_to_objects={
            "Movable": [
                ENTITIES["beaker"],
                ENTITIES["flask"],
                ENTITIES["magnet"],
            ],
            "Surface": [
                ENTITIES["table"],
                ENTITIES["goal_region"],
                ENTITIES["pour_region"],
                ENTITIES["beaker_region"],
            ],

        }



    )