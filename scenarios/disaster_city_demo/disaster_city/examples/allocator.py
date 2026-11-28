"""Minimal interchangeable baseline; replace choose() with your algorithm."""
from disaster_city.allocation import KnownScheduler


class Allocator:
    policy_name = 'example_nearest_quorum'
    def __init__(self, grid, cfg):
        self.baseline = KnownScheduler(grid, cfg)

    def choose(self, observation, feasible_actions):
        # Public state only. 0 keeps the current assignments.
        # feasible_actions maps action IDs to atomic robot/team proposals.
        return self.baseline.choose(observation)
