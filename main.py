import asyncio
import argparse
import cProfile
import signal

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='scenarios/simple/configs/grape.yaml', help='Path to the configuration file (default: --config=scenarios/simple/configs/config.yaml)')
parser.add_argument('--ns', type=str, default=None, help='Override agent namespace, e.g. --ns /Fire_UGV_2')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
if args.ns is not None:
    config['agent']['namespaces'] = args.ns
from modules.bt_runner import BTRunner
bt_runner = BTRunner(config)


async def loop():
    # ROS installs its own SIGINT handler; explicitly route both signals through
    # the runner so its goal cancellation and launch cleanup can finish.
    for signum in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(
            signum, lambda: setattr(bt_runner, 'running', False)
        )
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
            # A tick can complete without suspending (notably while IDLE).
            # Give asyncio a turn to dispatch signal callbacks in that case.
            await asyncio.sleep(0)
    finally:
        bt_runner.close()  # halt_tree() → cancel active ROS Action goals


if __name__ == "__main__":
    if config['bt_runner']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        try:
            asyncio.run(loop())
        except KeyboardInterrupt:
            pass  # bt_runner.close()는 loop() finally에서 이미 호출됨
