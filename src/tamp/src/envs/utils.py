from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field

import torch
from curobo.geom.types import Cuboid, Cylinder
from curobo.types.base import TensorDeviceType
from cutamp.envs import TAMPEnvironment
from cutamp.envs.utils import unit_quat
from cutamp.tamp_domain import HandEmpty, On, Poured, OnBeaker
from cutamp.utils.shapes import MultiSphere
from cutamp.envs import TAMPEnvironment

from envs.pouring import load_pouring_env
from envs.stirring import load_stirring_env


ENTITIES = {
    "table": Cuboid(name="table", pose=[0.0, 0.0, -0.01, *unit_quat], dims=[1.5, 1.5, 0.02], color=[235, 196, 145]),
    "stirrer": Cuboid(name="stirrer", pose=[0.3, 0.3, 0.0125, *unit_quat], dims=[0.1, 0.1, 0.03], color=[235, 196, 145]),

    # objects
    "beaker": Cuboid(name="beaker", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.05, 0.05, 0.12], color=[0, 0, 255]),
    "flask": Cuboid(name="flask", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.06, 0.06, 0.08], color=[0, 255, 0]),
    "magnet": Cuboid(name="magnet", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.045, 0.045, 0.03], color=[255, 0, 255]),

    # Reigion
    "goal_region": Cuboid(name="goal", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.1, 0.1, 0.01], color=[186, 255, 201]),
    "pour_region": Cuboid(name="beaker_region", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.06, 0.06, 0.0001], color=[255, 255, 255]),
    "beaker_region": Cuboid(name="beaker_region", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.06, 0.06, 0.0001], color=[255, 255, 255]),

    "obstacle_1": Cuboid(name="obstacle_1", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.07, 0.07, 0.1], color=[255, 0, 255]),
    "obstacle_2": Cuboid(name="obstacle_2", pose=[0.0, 0.0, 0.0, *unit_quat], dims=[0.07, 0.07, 0.1], color=[255, 0, 255]),
}


def load_demo_env(name: str) -> TAMPEnvironment:
    # Envs for SDL
    if name == "pouring":
        env = load_pouring_env()
    elif name == "stirring":
        env = load_stirring_env()
    else:
        raise ValueError(f"Unknown environment name: {name}")
    return env



class TAMPEnvManager:

    def __init__(self):

        self.entities = ENTITIES

        self.is_update_entities = False
        self.movables = []
        self.statics = []
        self.ex_collision = []


    def update_entities(
        self,
        poses: Dict[str, List[float]],
        movables: List[str] = None,
        statics: List[str] = None,
        ex_collision: List[str] = None,
    ):
        self.movables = []
        self.statics = []
        self.ex_collision = []

        if poses is not None:
            for name, pose in poses.items():
                self.entities[name.lower()].pose = pose

        if movables is not None:
            for name in movables:
                self.movables.append(self.entities[name.lower()])

        if statics is not None:
            for name in statics:
                self.statics.append(self.entities[name.lower()])

        if ex_collision is not None:
            for name in ex_collision:
                self.ex_collision.append(self.entities[name.lower()])

        self.is_update_entities = True


    def load_env(self, name: str) -> TAMPEnvironment:
        
        name = name.lower()

        if self.is_update_entities:

            if name == "pouring":

                self.entities["pour_region"].pose = [0.0, 0.5, 0.15, *unit_quat]
                self.entities["goal_region"].pose = [0.3, 0.0, 0.01, *unit_quat]

                env = TAMPEnvironment(
                    name=name,
                    movables=self.movables,
                    statics=self.statics,
                    ex_collision=self.ex_collision,
                    type_to_objects={
                        "Movable": self.movables,
                        "Surface": [self.entities["table"], self.entities["pour_region"], self.entities["goal_region"]],
                        "ExCollision": [self.entities["pour_region"]]
                    },
                    goal_state=frozenset(
                        { 
                            On.ground(self.entities["beaker"].name, self.entities["goal_region"].name), 
                            HandEmpty.ground(),
                            Poured.ground(self.entities["beaker"].name, self.entities["pour_region"].name),
                        }
                    )
                )

            elif name == "stirring":

                self.entities["beaker_region"].pose = [0.3, 0.3, 0.16, *unit_quat]
                self.entities["goal_region"].pose = [0.3, 0.3, 0.0125, *unit_quat]

                env = TAMPEnvironment(
                    name=name,
                    movables=self.movables,
                    statics=self.statics,
                    ex_collision=self.ex_collision,
                    type_to_objects={
                        "Movable": self.movables,
                        "Surface": [self.entities["table"], self.entities["stirrer"], self.entities["beaker_region"], self.entities["goal_region"]],
                        "ExCollision": [self.entities["beaker_region"]]
                    },
                    goal_state=frozenset(
                        {
                            HandEmpty.ground(),
                            On.ground(self.entities["flask"].name, self.entities["goal_region"].name), 
                            On.ground(self.entities["magnet"].name, self.entities["beaker_region"].name),
                            OnBeaker.ground(self.entities["magnet"].name, self.entities["beaker_region"].name), 
                        }
                    )
                )
            else:
                raise ValueError("name should be only 'pouring' or 'stirring'")
        else:
            raise ValueError("Do 'update_entities' methods before loading env")
        
        return env

