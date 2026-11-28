import json
from pathlib import Path
import pytest
from disaster_city.grid import CityGrid
from disaster_city.engine import Mission
from disaster_city.plugins import MaskedScheduler, load_policy
from disaster_city.allocation import KnownScheduler

PROFILE=Path(__file__).resolve().parents[1]/'disaster_city/profiles/junyong_10_damage'

def test_example_matches_baseline_and_rejects_invalid_action():
    cfg=json.loads((PROFILE/'scenario.json').read_text());grid=CityGrid(PROFILE/'city.pgm')
    m=Mission(cfg,grid)
    for r in m.robots:r['ready']=True
    obs=m.observe();p=MaskedScheduler(grid,cfg,'disaster_city.examples.allocator:Allocator')
    assert p.choose(obs)==KnownScheduler(grid,cfg).choose(obs)
    p.plugin.choose=lambda obs,options:999999
    with pytest.raises(ValueError,match='infeasible'):p.choose(obs)
    with pytest.raises(ValueError,match='not both'):load_policy(grid,cfg,'x:y','model.zip')
