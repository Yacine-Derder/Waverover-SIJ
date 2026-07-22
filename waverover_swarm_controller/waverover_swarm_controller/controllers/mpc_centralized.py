"""Centralized receding-horizon convex controller."""

from .base import minimum_lookahead, replace_first_future_points
from .convex import ConvexController
from ..models import ControllerResult


class CentralizedMpcController(ConvexController):
    def compute(self, snapshot):
        result = super().compute(snapshot)
        lookahead_setpoints = {}
        for robot_id in sorted(result.setpoints):
            lookahead_setpoints[robot_id] = minimum_lookahead(
                snapshot.robots[robot_id],
                result.setpoints[robot_id],
                self.config.controller.minimum_mpc_lookahead_m,
                self.config.controller.mpc_max_step_m,
            )
        setpoints = lookahead_setpoints
        predicted_paths = replace_first_future_points(
            result.predicted_paths, setpoints
        )
        return ControllerResult(
            setpoints=setpoints,
            target_assignments=result.target_assignments,
            predicted_paths=predicted_paths,
            selected_edges=result.selected_edges,
            solver_status=result.solver_status,
            solve_duration_sec=result.solve_duration_sec,
            diagnostic=(
                'Centralized MPC publishes only the first future carrot; '
                + result.diagnostic
            ),
            created_at=result.created_at,
            target_epoch=snapshot.target_epoch,
            optimization_mode=result.optimization_mode,
            controller_diagnostics=result.controller_diagnostics,
        )

    def compute_recovery(self, snapshot):
        result = ConvexController.compute_recovery(self, snapshot)
        lookahead = {
            robot_id: minimum_lookahead(
                snapshot.robots[robot_id], result.setpoints[robot_id],
                self.config.controller.minimum_mpc_lookahead_m,
                self.config.controller.mpc_max_step_m,
            )
            for robot_id in sorted(result.setpoints)
        }
        return ControllerResult(
            setpoints=lookahead,
            target_assignments=result.target_assignments,
            predicted_paths=replace_first_future_points(
                result.predicted_paths, lookahead
            ),
            selected_edges=result.selected_edges,
            solver_status=result.solver_status,
            solve_duration_sec=result.solve_duration_sec,
            diagnostic=result.diagnostic,
            created_at=result.created_at,
            target_epoch=result.target_epoch,
            optimization_mode='recovery_mpc',
            controller_diagnostics=result.controller_diagnostics,
        )

    @property
    def horizon_steps(self):
        return self.config.controller.mpc_horizon
