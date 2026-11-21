#!/usr/bin/env python3
"""Run the existing exploration BT without installing Webots on the LIMO."""
import sys

from webots_ros2_limo.exploration_ros import robot_main, team_main


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('team', 'robot'):
        raise SystemExit('Usage: real_single_bt.py team|robot [ROS arguments]')
    (team_main if sys.argv[1] == 'team' else robot_main)(sys.argv[2:])


if __name__ == '__main__':
    main()
