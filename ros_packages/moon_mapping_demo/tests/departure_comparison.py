#!/usr/bin/env python3
"""같은 WBT·센서·ROI에서 기존/선발 2대 정책을 순서대로 실행하고 비교한다.

Webots 물리 실행이 아닌 이상 기구학·모의 센서 검사다. 실패도 결과로 보존하며
기존 결과 폴더를 재사용하지 않는다. 실험 판정은 comparison.json의 각 행을 본다.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import threading

from bt_kinematic_smoke import run
from moon_mapping_demo.experiment import load_experiment, make_field
from moon_mapping_demo.exploration_planning import Candidate, plan_batch
from moon_mapping_demo.metrics import atomic_json


def frozen_failure(root, share):
    old=Path('results/bt_wide_final_v2_20261122/bt_kinematic_validation.json')
    if not old.exists(): return {'available':False}
    old_report=json.loads(old.read_text())
    problem=json.loads((Path(old_report['session_directory'])/'round_0001/plan_0006/attempt_000/problem.json').read_text())
    cfg,env,pcfg,share=load_experiment(share/'config/exploration_wide.yaml')
    field=make_field(cfg,env,pcfg,share)
    poses={r:tuple(p) for r,p in old_report['last_poses'].items()}
    goals=[Candidate(**g) for g in problem['metadata']['goals']]
    result={}
    for policy in ('simultaneous','staggered_pairs'):
        value=plan_batch(field,pcfg,poses,goals,problem['t0_sim_seconds'],old_report['sim_time']-.1,
            10.,cfg['robots'],root/'frozen_failure'/policy,threading.Event(),
            {**cfg['exploration'],'departure':{'mode':policy}},True)
        result[policy]={'status':value['status'],'error':value['error'],
            'costs':value['attempts'][0]['costs'] if value['attempts'] else None}
    t0=problem['t0_sim_seconds'];p=poses['robot3'][:2]
    result.update(available=True,original_report=str(old),common_t0=t0,
        robot3_static_forbidden=field.static_blocked_segment(p,p,.26),
        robot3_at_t0_forbidden=field.blocked_segment(p,p,t0,t0,.26),
        interpretation='이미 robot3의 공통 t0 출발점이 시간 금지인 상태는 배정 이후 출발 시차만으로 복구할 수 없음')
    return result


def summarize(report, path, profile, policy):
    session=Path(report['session_directory']);metrics=json.loads((session/'metrics.json').read_text())
    problems=sorted(session.glob('round_0000/plan_*/attempt_000/problem.json'))
    initial=json.loads(problems[0].read_text()) if problems else None
    failures=Counter()
    for f in session.glob('plan_failure_*.json'):
        d=json.loads(f.read_text())
        failures[d['status']]+=1
        for attempt in d.get('attempts',[]):
            for trial in attempt.get('departure',{}).get('attempts',[]):
                failures[trial.get('failure') or 'cancelled_or_unfinished_trial']+=1
    return {'profile':profile,'policy':policy,'passed':report['passed'],'phase':report['phase'],
        'reason':report['reason'],'elapsed_sim_seconds':report['sim_time']-1.,
        'elapsed_wall_seconds':report['elapsed_wall_seconds'],'rounds_completed':report['rounds_completed'],
        'coverage_percent':100*report['observation']['coverage_fraction'],
        'fixed_denominator_m2':report['observation']['denominator_m2'],'replans':report['replans'],
        'minimum_sampled_distance_m':report['minimum_sampled_distance_m'],
        'initial_problem':initial,'plan_costs':metrics['plan_costs'],
        'actual_wait_sim_seconds':{r:sum(v.values()) for r,v in metrics['waiting_sim_seconds_by_reason'].items()},
        'departure_wait_sim_seconds':{r:v.get('departure_wait',0.) for r,v in metrics['waiting_sim_seconds_by_reason'].items()},
        'explore_participants':sorted({r for e in report['dispatches'] for r,g in e['assigned'].items() if g['purpose'] in ('fixed','explore')}),
        'failure_counts':dict(failures),'report':str(path),'session_directory':str(session)}


def plots(root, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
    for ax,profile in zip(axes,('short','wide')):
        for row in rows:
            if row['profile']!=profile: continue
            data=json.loads((Path(row['session_directory'])/'metrics.json').read_text())
            samples=data['samples']
            ax.plot([s['elapsed_sim_seconds'] for s in samples],[s['coverage_percent'] for s in samples],
                    label=f"{row['policy']} / {row['rounds_completed']}회차")
        ax.set(xlabel='경과 시뮬레이션 시간 (초)',ylabel='관측률 (%)',title=profile,ylim=(0,100))
        ax.grid(alpha=.3);ax.legend()
    fig.suptitle('동일 WBT·고정 ROI 비교 — 모의 센서 / 이상 기구학 (물리 실행 아님)')
    fig.savefig(root/'coverage_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
    for ax,row in zip(axes,[r for r in rows if r['profile']=='fixed']):
        report=json.loads(Path(row['report']).read_text())
        if report['departures']:
            first=report['departures'][0]['plan_id']
            records=[d for d in report['departures'] if d['plan_id']==first]
            for i,d in enumerate(records):
                planned=d['planned_translation_start_sim_seconds'];actual=d['actual_translation_start_sim_seconds']
                if planned is not None:
                    ax.barh(i,planned-d['t0_sim_seconds'],left=d['t0_sim_seconds'],height=.5,color='gold',label='이동 전 대기·회전' if i==0 else '')
                    ax.scatter(planned,i,marker='|',s=180,color='blue',label='예정 이동 시작' if i==0 else '')
                if actual is not None:
                    ax.scatter(actual,i,marker='x',s=40,color='red',label='odom 이동 시작' if i==0 else '')
            ax.set_yticks(range(len(records)),[d['robot']+' '+('후발' if d['role']=='follow' else '선발') for d in records])
        ax.set(title=row['policy'],xlabel='절대 시뮬레이션 시각 (초)');ax.grid(alpha=.3);ax.legend(loc='upper left')
    fig.suptitle('4대 고정 배정: 계획 기준 t0와 예정·관측 이동 시작의 구분 (모의 검사)')
    fig.savefig(root/'departure_timeline.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output
    root.mkdir(parents=True,exist_ok=False)
    share=Path(__file__).resolve().parents[1]
    rows=[]
    for profile,mode,config in [('fixed','fixed','exploration_short.yaml'),
        ('short','explore','exploration_short.yaml'),('wide','explore','exploration_wide.yaml')]:
        for policy in ('simultaneous','staggered_pairs'):
            folder=root/(profile+'_'+policy)
            print(f'검사 시작: {profile} / {policy}',flush=True)
            run(share/'config'/config,folder,mode,departure_policy=policy)
            path=folder/'bt_kinematic_validation.json';report=json.loads(path.read_text())
            rows.append(summarize(report,path,profile,policy))
            atomic_json(root/'comparison_progress.json',rows)
    equality={}
    for profile in ('fixed','short','wide'):
        a,b=[r for r in rows if r['profile']==profile]
        x,y=a['initial_problem'],b['initial_problem']
        equality[profile]=bool(x and y and all(x[k]==y[k] for k in
            ('robot_ids','goal_ids','costs','t0_sim_seconds','time_step_seconds','assignment')))
    result={'kind':'정책 비교; 실제 Webots 아님','physical_webots_run':False,'rows':rows,
        'initial_hungarian_problem_equal':equality,'frozen_failure':frozen_failure(root,share),
        'optimization_scope':'선발·순서·시차는 유한 휴리스틱이며 공동 실행·전체 탐사의 최적성 보장 없음'}
    atomic_json(root/'comparison.json',result)
    columns=['profile','policy','passed','phase','reason','elapsed_sim_seconds','elapsed_wall_seconds',
        'rounds_completed','coverage_percent','fixed_denominator_m2','replans','minimum_sampled_distance_m']
    with (root/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    plots(root,rows)
    print(json.dumps({'initial_hungarian_problem_equal':equality,
        'results':[{k:r[k] for k in columns} for r in rows]},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
