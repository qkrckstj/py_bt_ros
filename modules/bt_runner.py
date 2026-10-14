"""Build the agent's XML tree and tick it at the configured rate."""
import asyncio
from pathlib import Path

from modules.agent import Agent


class BTRunner:
    def __init__(self, config):
        self.bt_tick_rate = config['bt_runner']['bt_tick_rate']
        self.agent = Agent()
        root = Path(__file__).resolve().parents[1]
        scenario_path = Path(*config['scenario'].split('.'))
        tree_path = root / scenario_path / config['agent']['behavior_tree_xml']
        self.agent.create_behavior_tree(str(tree_path))

    async def step(self):
        await self.agent.run_tree()
        await asyncio.sleep(1.0 / self.bt_tick_rate)

    def close(self):
        self.agent.close()
