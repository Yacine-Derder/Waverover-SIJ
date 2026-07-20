"""Deterministic, monotonic runtime target-priority state."""

import random

from .models import TargetState


class TargetManager:
    """Own dynamic weights and return immutable per-cycle target snapshots."""

    def __init__(self, targets, config, start_time=0.0):
        self._base = {target.target_id: target for target in targets}
        self._ids = tuple(sorted(self._base))
        if not self._ids:
            raise ValueError('TargetManager requires at least one target.')
        self.config = config
        self._rng = random.Random(int(config.seed))
        self.start_time = float(start_time)
        self.epoch = 0
        self.switch_reason = 'experiment_start'
        requested = config.initial_priority_target_id
        self.priority_target_id = (
            str(requested) if requested is not None
            else self._rng.choice(self._ids)
        )
        if self.priority_target_id not in self._base:
            raise ValueError('Unknown initial priority target %s.' % requested)

    def _switch_once(self):
        if len(self._ids) == 1:
            return
        choices = tuple(
            target_id for target_id in self._ids
            if target_id != self.priority_target_id
        )
        self.priority_target_id = self._rng.choice(choices or self._ids)

    def advance(self, now):
        elapsed = max(0.0, float(now) - self.start_time)
        required_epoch = int(elapsed // self.config.switch_period_sec)
        changed = required_epoch > self.epoch
        while self.epoch < required_epoch:
            self._switch_once()
            self.epoch += 1
        if changed:
            self.switch_reason = 'period_boundary'
        return changed

    def snapshot(self, now):
        self.advance(now)
        targets = tuple(
            TargetState(
                target_id=target_id,
                x=self._base[target_id].x,
                y=self._base[target_id].y,
                weight=(
                    self.config.priority_weight
                    if target_id == self.priority_target_id
                    else self.config.background_weight
                ),
                is_priority=target_id == self.priority_target_id,
            )
            for target_id in self._ids
        )
        epoch_start = self.start_time + self.epoch * self.config.switch_period_sec
        return targets, {
            'priority_target_id': self.priority_target_id,
            'target_epoch': self.epoch,
            'target_epoch_started_at': epoch_start,
            'target_epoch_elapsed_sec': max(0.0, float(now) - epoch_start),
            'next_switch_at': epoch_start + self.config.switch_period_sec,
            'seconds_until_switch': max(
                0.0, epoch_start + self.config.switch_period_sec - float(now)
            ),
            'target_selection_seed': int(self.config.seed),
            'switch_reason': self.switch_reason,
        }
