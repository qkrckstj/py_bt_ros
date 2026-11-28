"""Public-state allocation plugin interface, shared by Webots and evaluation."""
import importlib
from .allocation import TeamActions, KnownScheduler


def load_class(spec):
    module, separator, name = spec.partition(':')
    if not separator:
        raise ValueError('Allocator must be module:ClassName')
    return getattr(importlib.import_module(module), name)


class MaskedScheduler:
    """Wrap a trusted plugin choosing a feasible team-action ID."""
    def __init__(self, grid, cfg, spec):
        self.actions = TeamActions(grid, cfg)
        self.plugin = load_class(spec)(grid, cfg)
        self.policy_name = getattr(self.plugin, 'policy_name', spec)
        self.last_action = None

    def choose(self, obs):
        options = self.actions.candidates(obs)
        action = int(self.plugin.choose(obs, options))
        if action not in options:
            raise ValueError(f'{self.policy_name} returned infeasible action {action}')
        self.last_action = action
        return action

    def decide(self, obs):
        action = self.choose(obs)
        return self.actions.candidates(obs)[action] + self.actions.safety(obs)


def load_policy(grid, cfg, allocator=None, model=None):
    if allocator and model:
        raise ValueError('Select CITY_ALLOCATOR or CITY_PPO_MODEL, not both')
    if allocator:
        return MaskedScheduler(grid, cfg, allocator)
    if model:
        from .rl.runtime_policy import PpoScheduler
        return PpoScheduler(grid, cfg, model)
    if cfg.get('tasks_known'):
        return KnownScheduler(grid, cfg)
    from .policy import Baseline
    return Baseline(grid, cfg['speed_mps'])
