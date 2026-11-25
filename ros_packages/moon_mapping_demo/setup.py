from pathlib import Path
from setuptools import find_packages, setup
assets=[]
for directory in ('config','launch','worlds','protos','controllers','LICENSES'):
 for p in Path(directory).rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts:
   assets.append(('share/moon_mapping_demo/'+str(p.parent),[str(p)]))
setup(name='moon_mapping_demo',version='0.1.0',packages=find_packages(),
 package_data={'moon_mapping_demo.vendor.team_bt':['LICENSE','UPSTREAM.json']},
 data_files=[('share/ament_index/resource_index/packages',['resource/moon_mapping_demo']),('share/moon_mapping_demo',['package.xml','README.md','THIRD_PARTY_NOTICES.md'])]+assets,
 install_requires=['setuptools','numpy','PyYAML','scipy','matplotlib'],entry_points={'console_scripts':['moon_live_mapping=moon_mapping_demo.live_mapping:main','moon_ready=moon_mapping_demo.ready:main','moon_shadow_plan=moon_mapping_demo.experiment:main','moon_shadow_mission=moon_mapping_demo.shadow_mission:main','moon_exploration=moon_mapping_demo.exploration_ros:main','moon_exploration_plot=moon_mapping_demo.metrics:main']})
