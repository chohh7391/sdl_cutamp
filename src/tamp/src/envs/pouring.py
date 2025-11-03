from typing import Optional, Dict, List, Any
import torch
from curobo.geom.types import Cuboid, Cylinder, Obstacle
from curobo.types.base import TensorDeviceType
from cutamp.envs import TAMPEnvironment
from cutamp.envs.utils import unit_quat
from cutamp.tamp_domain import HandEmpty, On, Poured, OnBeaker
from cutamp.utils.shapes import MultiSphere


def load_pouring_env(
    entities: Dict[str, Any],
    movables: List[Obstacle],
    statics: List[Obstacle],
    ex_collision: List[Obstacle],
    tensor_args: TensorDeviceType = TensorDeviceType(),
) -> TAMPEnvironment:
    """Pick-and-place environment with a cylindrical beaker and small MultiSphere near goal."""

    entities["pour_region"].pose = entities["flask"].pose.copy() # [0.0, 0.5, 0.15, *unit_quat]
    entities["pour_region"].pose[2] += 0.15
    entities["goal_region"].pose = [0.3, 0.0, 0.01, *unit_quat] # custom beaker placepose

    env = TAMPEnvironment(
        name="pouring",
        movables=movables,
        statics=statics,
        ex_collision=ex_collision,
        type_to_objects={
            "Movable": movables,
            "Surface": [entities["table"], entities["pour_region"], entities["goal_region"]],
            "ExCollision": [entities["pour_region"]]
        },
        goal_state=frozenset(
            { 
                On.ground(entities["beaker"].name, entities["goal_region"].name), 
                HandEmpty.ground(),
                Poured.ground(entities["beaker"].name, entities["pour_region"].name),
            }
        )
    )

    return env