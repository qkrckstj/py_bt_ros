"""Deploy the same masked policy and public observation used in training."""
import hashlib
from pathlib import Path
import torch
from sb3_contrib import MaskablePPO
from disaster_city.allocation import TeamActions
from .env import encode


class PpoScheduler:
    def __init__(self, grid, cfg, model_path):
        torch.set_num_threads(2)
        self.actions = TeamActions(grid, cfg)
        self.model = MaskablePPO.load(model_path, device='cpu')
        if self.model.action_space.n != self.actions.size:
            raise ValueError('PPO model action space does not match scenario')
        self.model_path = str(Path(model_path).resolve())
        self.model_sha256 = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.policy_name = 'PPO | learned allocation'
        self.last_action = None

    def decide(self, obs):
        import numpy as np
        options = self.actions.candidates(obs)
        mask = np.zeros(self.actions.size, dtype=bool)
        mask[list(options)] = True
        state = encode(obs, self.actions)
        if state.shape != self.model.observation_space.shape:
            raise ValueError('PPO model observation space does not match scenario')
        action, _ = self.model.predict(state, deterministic=True, action_masks=mask)
        self.last_action = int(action)
        return options[self.last_action] + self.actions.safety(obs)
