"""CPU MaskablePPO training and reproducible paired baseline evaluation."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import torch
from sb3_contrib import MaskablePPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from .env import AllocationEnv, DEFAULT_PROFILE
from disaster_city.allocation import KnownScheduler


def evaluate(profile, model, seeds, randomize, allocator=None):
    rows=[];env=AllocationEnv(profile,randomize=randomize)
    for seed in seeds:
        obs,_=env.reset(seed=seed)
        from disaster_city.plugins import MaskedScheduler
        baseline=MaskedScheduler(env.grid,env.cfg,allocator) if allocator else KnownScheduler(env.grid,env.cfg)
        started=time.monotonic()
        while env.mission.phase=='RUNNING':
            if model is None:action=baseline.choose(env.mission.observe())
            else:action,_=model.predict(obs,deterministic=True,action_masks=env.action_masks())
            obs,_,_,_,info=env.step(int(action))
        rows.append(dict(seed=seed,**info,wall_seconds=time.monotonic()-started,
                         events=env.mission.events,initial_battery=env.cfg['initial_battery']))
    env.close();return rows


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','train','compare'])
    p.add_argument('--profile',type=Path,default=DEFAULT_PROFILE);p.add_argument('--steps',type=int,default=200000)
    p.add_argument('--seed',type=int,default=0);p.add_argument('--count',type=int,default=10)
    p.add_argument('--out',type=Path,default=Path('disaster_city/rl/results'))
    p.add_argument('--model',type=Path);p.add_argument('--randomize',action='store_true')
    p.add_argument('--allocator',help='Custom baseline module:ClassName (choose(public_state, feasible_actions))')
    p.add_argument('--resume',type=Path,help='Continue a saved policy for --steps additional transitions')
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);torch.set_num_threads(2)
    fingerprint=hashlib.sha256((a.profile/'scenario.json').read_bytes()+(a.profile/'city.pgm').read_bytes()).hexdigest()
    if a.mode=='train':
        env=Monitor(AllocationEnv(a.profile,randomize=True),str(a.out/'monitor.csv'))
        model=MaskablePPO('MlpPolicy',env,learning_rate=3e-4,n_steps=512,batch_size=128,n_epochs=5,
                         gamma=.999,gae_lambda=.95,ent_coef=.01,policy_kwargs=dict(net_arch=[128,128]),
                         device='cpu',verbose=1,seed=a.seed)
        if a.resume:
            model=MaskablePPO.load(a.resume,env=env,device='cpu')
            model.set_random_seed(a.seed)
        initial_steps=model.num_timesteps
        started=time.monotonic()
        model.learn(total_timesteps=a.steps,reset_num_timesteps=not bool(a.resume),callback=CheckpointCallback(save_freq=8192,save_path=str(a.out/'checkpoints')))
        model.save(a.out/'ppo_allocation')
        (a.out/'training.json').write_text(json.dumps(dict(algorithm='MaskablePPO',seed=a.seed,
            requested_steps=a.steps,actual_steps=model.num_timesteps,wall_seconds=time.monotonic()-started,
            initial_steps=initial_steps,resumed_from=str(a.resume.resolve()) if a.resume else None,
            profile=str(a.profile.resolve()),profile_sha256=fingerprint,
            observation_size=int(env.observation_space.shape[0]),actions=int(env.action_space.n),
            randomization=dict(battery=[50,100],task_multiplier=[.8,1.2],fire_growth=json.loads((a.profile/'scenario.json').read_text()).get('fire_growth_range',[.05,.15])),
            reward='-elapsed_seconds/100; horizon penalty for failure; potential shaping gamma=0.999'),indent=2))
        env.close();return
    rows={'profile':str(a.profile.resolve()),'profile_sha256':fingerprint,'randomize':a.randomize,
          'baseline_policy':a.allocator or KnownScheduler.policy_name,
          'baseline':evaluate(a.profile,None,range(a.seed,a.seed+a.count),a.randomize,a.allocator)}
    if a.mode=='compare':
        if a.model is None:p.error('--model required for compare')
        model=MaskablePPO.load(a.model,device='cpu')
        rows['ppo']=evaluate(a.profile,model,range(a.seed,a.seed+a.count),a.randomize)
        rows['model']=str(a.model.resolve())
        rows['model_sha256']=hashlib.sha256(a.model.read_bytes()).hexdigest()
    path=a.out/(a.mode+'.json');path.write_text(json.dumps(rows,indent=2))
    for name in ('baseline','ppo'):
        if name in rows:
            data=rows[name];good=[r['time_s'] for r in data if r['phase']=='COMPLETE']
            print(name,'success',len(good),'/',len(data),'successful mean',sum(good)/len(good) if good else None,
                  'times',[r['time_s'] for r in data],flush=True)
    print('Saved',path,flush=True)

if __name__=='__main__':main()
