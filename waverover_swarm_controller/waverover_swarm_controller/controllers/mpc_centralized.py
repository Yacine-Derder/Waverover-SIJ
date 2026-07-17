"""Centralized receding-horizon convex controller."""

from ..models import ControllerResult
from .base import minimum_lookahead
from .convex import ConvexController


class CentralizedMpcController(ConvexController):
    def compute(self, snapshot):
        result = super().compute(snapshot)
        setpoints = {}
        for robot_id in sorted(result.setpoints):
            setpoints[robot_id] = minimum_lookahead(
                snapshot.robots[robot_id],
                result.setpoints[robot_id],
                self.config.controller.minimum_mpc_lookahead_m,
                self.config.controller.mpc_max_step_m,
            )
        return ControllerResult(
            setpoints=setpoints,
            predicted_paths=result.predicted_paths,
            selected_edges=result.selected_edges,
            solver_status=result.solver_status,
            solve_duration_sec=result.solve_duration_sec,
            diagnostic=(
                'Centralized MPC publishes only the first future carrot; '
                + result.diagnostic
            ),
            created_at=result.created_at,
        )

    @property
    def horizon_steps(self):
        return self.config.controller.mpc_horizon
