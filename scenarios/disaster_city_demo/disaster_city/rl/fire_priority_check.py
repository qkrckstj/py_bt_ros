"""Paired diagnostic: same baseline, with only early fire dispatch withheld.

The delayed policy is an evaluation control, never a training restriction.
Robots remain free to perform other jobs during the withholding interval.
"""
import argparse
import hashlib
import json
from pathlib import Path

from disaster_city.allocation import KnownScheduler, TeamActions
from .env import AllocationEnv


class DelayedFireActions(TeamActions):
    def __init__(self, grid, cfg, delay):
        super().__init__(grid, cfg)
        self.delay = delay

    def candidates(self, obs):
        options = super().candidates(obs)
        if obs['time_s'] >= self.delay:
            return options
        return {a: ps for a, ps in options.items() if not any(
            role == 'work' and obs['tasks'][task]['kind'] == 'fire'
            for _, role, _, task, _ in ps)}


def run(profile, seed, randomize, delay):
    env = AllocationEnv(profile, randomize=randomize)
    env.reset(seed=seed)
    scheduler = KnownScheduler(env.grid, env.cfg)
    scheduler.actions = DelayedFireActions(env.grid, env.cfg, delay)
    while env.mission.phase == 'RUNNING':
        _, _, _, _, info = env.step(scheduler.choose(env.mission.observe()))
    fire_ids = {jid for jid, job in env.mission.jobs.items() if job['kind'] == 'fire'}
    first_dispatch = next((e['time_s'] for e in env.mission.events
                           if e['kind'] == 'assignment' and e.get('role') == 'work'
                           and e.get('task') in fire_ids), None)
    result = dict(seed=seed, fire_dispatch_not_before_s=delay,
                  fire_first_dispatch_s=first_dispatch, **info,
                  initial_battery=env.cfg['initial_battery'], events=env.mission.events)
    env.close()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile', type=Path, required=True)
    p.add_argument('--seed', type=int, default=1000)
    p.add_argument('--count', type=int, default=20)
    p.add_argument('--delay', type=float, default=30.)
    p.add_argument('--randomize', action='store_true')
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    pairs = []
    for seed in range(args.seed, args.seed + args.count):
        early = run(args.profile, seed, args.randomize, 0.)
        delayed = run(args.profile, seed, args.randomize, args.delay)
        pairs.append(dict(seed=seed, early=early, delayed=delayed))
        print(seed, early['phase'], early['time_s'], delayed['phase'], delayed['time_s'], flush=True)
    result = dict(profile=str(args.profile.resolve()), randomize=args.randomize,
                  baseline_policy=KnownScheduler.policy_name,
                  profile_sha256=hashlib.sha256((args.profile/'scenario.json').read_bytes()
                                               +(args.profile/'city.pgm').read_bytes()).hexdigest(),
                  intervention='withhold fire dispatch only; otherwise identical baseline', pairs=pairs)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
