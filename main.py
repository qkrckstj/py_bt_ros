"""Entry point for the homework behavior tree process."""
import argparse
import asyncio
from pathlib import Path

from modules.utils import set_config


async def run(config):
    # Agent imports the configuration, so import only after set_config().
    from modules.bt_runner import BTRunner

    runner = BTRunner(config)
    try:
        while True:
            await runner.step()
    finally:
        runner.close()


def main():
    parser = argparse.ArgumentParser(description='py_bt_ros photo mission')
    parser.add_argument('--config', default=str(Path(__file__).with_name('config.yaml')))
    args = parser.parse_args()
    set_config(args.config)
    from modules.utils import config

    try:
        asyncio.run(run(config))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
