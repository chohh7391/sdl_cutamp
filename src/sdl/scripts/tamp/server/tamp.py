from typing import Optional, List

from cutamp.algorithm import run_cutamp
from cutamp.config import TAMPConfiguration, validate_tamp_config
from cutamp.constraint_checker import ConstraintChecker
from cutamp.cost_reduction import CostReducer
from cutamp.envs import TAMPEnvironment
from cutamp.scripts.utils import (
    default_constraint_to_mult,
    default_constraint_to_tol,
    setup_logging,
    get_tetris_tuned_constraint_to_mult,
)
from cutamp.task_planning.base_structs import State
import logging
from cutamp.cost_reduction import CostReducer
from cutamp.envs.utils import TAMPEnvironment


class TAMP:

    def __init__(
        self,
        config: TAMPConfiguration,
        env: TAMPEnvironment,
        use_tetris_tuned_weights: bool = None,
    ):
        validate_tamp_config(config)

        self.config = config
        self.env = env

        setup_logging()

        self.use_tetris_tuned_weights = use_tetris_tuned_weights

        self.constraint_to_mult = (
            get_tetris_tuned_constraint_to_mult() if self.use_tetris_tuned_weights else default_constraint_to_mult.copy()
        )
        self.cost_reducer = CostReducer(self.constraint_to_mult)
        self.constraint_checker = ConstraintChecker(default_constraint_to_tol.copy())
        self._log = logging.getLogger(__name__)

        self.curobo_plan = None
        self.total_num_satisfying = None


    def plan(
        self,
        q_init: Optional[List[float]] = None,
        experiment_id: Optional[str] = None
    ):
        self.curobo_plan, self.total_num_satisfying = run_cutamp(
            env=self.env,
            config=self.config,
            cost_reducer=self.cost_reducer,
            constraint_checker=self.constraint_checker,
            q_init=q_init,
            experiment_id=experiment_id
        )

        return self.curobo_plan, self.total_num_satisfying


    def update_config(self, config: TAMPConfiguration):
        self.config = config


    def update_env(self, ids: List[int], poses: List[List[float]]):

        for id, pose in zip(ids, poses):
            self.env.movables[id].pose = pose

    def update_goal_state(self, goal_state: State):
        self.env.goal_state = goal_state