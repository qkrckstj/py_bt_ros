#!/usr/bin/env python3
"""이른 대피 단계별 저장 결과를 재실행 없이 비교한다. exit0은 집계 완료만 뜻한다."""
import argparse
import csv
import json
from pathlib import Path

from moon_mapping_demo.metrics import atomic_json


CASES=[
 ('baseline_fixed','fixed','기존'),('final_fixed','fixed','복수 후보'),
 ('baseline_short','short','기존'),('final_short','short','복수 후보'),
 ('baseline_wide','wide','기존'),('stage1_early_wide','wide','① 이른 대피'),
 ('stage2_ranked_goals_wide','wide','② 관측·후속 시간'),
 ('stage3_margin_choice_wide','wide','③ 복수 후보'),
 ('stage4_margin_choice_wide','wide','④ 방향 A* 비교'),
 ('candidates_fixed_delay2','fixed_delay','복수 후보·2초 지연'),
 ('heading_fixed_delay2_corrected','fixed_delay','방향 A*·2초 지연'),
 ('candidates_fixed_delay6','fixed_delay','복수 후보·6초 지연'),
 ('candidates_short_delay2','short_delay','복수 후보·2초 지연')]


def classify(d):
    if d['phase']=='COMPLETE' and d['reason'] in ('fixed_goals_observed','coverage_target'):
        return '목표 달성'
    if d['phase']=='PARTIAL_COMPLETE':
        return '시간 제한 부분 완료' if d['reason']=='sim_duration_limit' else '후속 계획 탐색 실패 부분 종료'
    if 'budget' in d['reason'].lower():return '계산 기한 초과·안전 정지'
    return '안전·계획 실패 정지'


def run(root,output):
    if output.exists():raise FileExistsError('기존 집계 보존: 새 output 폴더를 지정하세요.')
    output.mkdir(parents=True);summaries=[];curves={}
    for name,profile,label in CASES:
        path=root/name/'bt_kinematic_validation.json'
        if not path.exists():continue
        d=json.loads(path.read_text())
        m=json.loads(next((root/name).glob('*/metrics.json')).read_text())
        end=d['sim_time']-m['started_sim_seconds']
        events=m['events'];jobs=[e for e in events if e['event']=='planning_timing']
        starts=[]
        for f in sorted((root/name).rglob('problem.json')):
            p=json.loads(f.read_text())
            for robot,row in p['metadata'].get('pair_statuses',{}).items():
                if row and all(v=='start_forbidden' for v in row.values()):
                    starts.append({'robot':robot,'t0':p['t0_sim_seconds'],'problem':str(f),
                        'xy':p['metadata']['all_robot_positions'][robot],
                        'cause':'출발 시각의 몸체 시간 금지. 원인 재검사는 geometry/failure_diagnosis 참조'})
        matched={}
        for t in (60.,75.,120.,180.,300.,600.):
            values=[s for s in m['samples'] if s['elapsed_sim_seconds']<=t+1e-8]
            sample=values[-1] if values and end>=t-1e-8 else None
            matched[str(int(t))]=({'coverage_percent':sample['coverage_percent'],
                'sample_elapsed_sim_seconds':sample['elapsed_sim_seconds']} if sample else None)
        first=d['dispatches'][0] if d['dispatches'] else None
        summary={'case':name,'profile':profile,'label':label,'phase':d['phase'],'reason':d['reason'],
            'outcome':classify(d),'harness_passed':d['passed'],'elapsed_sim_seconds':end,
            'elapsed_monotonic_wall_seconds':d['elapsed_wall_seconds'],'rounds_completed':d['rounds_completed'],
            'coverage_percent':100*d['observation']['coverage_fraction'],'fixed_denominator_m2':d['observation']['denominator_m2'],
            'replans':d['replans'],'dispatch_count':d['dispatch_count'],'matched_elapsed_sim':matched,
            'first_dispatch':first,'start_forbidden_null_row_occurrences':len(starts),
            'planning_statuses':{s:sum(j.get('status')==s for j in jobs) for s in sorted({j.get('status','미상') for j in jobs})},
            'minimum_sampled_distance_m':d['minimum_sampled_distance_m'],
            'injected_planning_delay_sim_seconds':d.get('injected_planning_delay_sim_seconds',0.),
            'first_risk_prediction':next((e for e in events if e['event']=='risk_first_prediction'),None),
            'first_early_escape':next((e for e in events if e['event']=='early_escape_prediction'),None),
            'planning_sim_clock_paused':d['planning_sim_clock_paused'],'physical_webots_run':False}
        summaries.append(summary);curves[name]=m['samples']
        atomic_json(output/(name+'_details.json'),{
            'risk_first_predictions':[e for e in events if e['event']=='risk_first_prediction'],
            'early_escape_predictions':[e for e in events if e['event']=='early_escape_prediction'],
            'planning_stop_checks':[e for e in events if e['event']=='planning_stop_check'],
            'planning_timing':jobs,'start_forbidden_null_rows':starts,
            'accepted_plan_costs':m['plan_costs'],'departures':m['departures'],
            'actual_waiting_seconds_by_reason':m['waiting_sim_seconds_by_reason'],
            'distance_m_by_robot':m['distance_m_by_robot'],
            'rotation_comparison_scope':'계획 turn_ticks는 취소된 경로의 미실행 회전도 포함. 실제 turning 시간·출발 시각은 별도이며 동일량으로 취급하지 않음.'})
    payload={'scope':'동일 WBT/초기 위치/절대 시작 sim=1/72광선·0.5초 합성센서/프로필별 고정 ROI. 실제 센서·물리 검증 아님.',
        'exit0_means':'집계 산출물 생성만 완료. harness_passed도 전체 기간/85% 달성을 뜻하지 않음.',
        'goal_changes':'②부터 안전 여유 후보 순위·최초 목표/비용 변경. ④는 방향·회전 시간 포함 정수 비용 모델로 변경.',
        'matching_rule':'종료 전 표본 중 해당 경과 sim 시각 이하의 최신 표본. 종료 후 값은 null; 보간·외삽 없음.',
        'default':'simultaneous + candidates. heading은 비교 옵션; 전체 탐사 최적성 없음.',
        'cases':summaries}
    atomic_json(output/'comparison.json',payload)
    columns=['case','profile','outcome','elapsed_sim_seconds','coverage_percent','rounds_completed','replans',
             'start_forbidden_null_row_occurrences','elapsed_monotonic_wall_seconds','reason']
    with (output/'comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
        for s in summaries:w.writerow({k:s[k] for k in columns})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for s in summaries:
        if s['profile']!='wide':continue
        samples=curves[s['case']]
        axes[0].plot([r['elapsed_sim_seconds'] for r in samples],[r['coverage_percent'] for r in samples],label=s['label'])
        axes[1].plot([r['elapsed_wall_seconds'] for r in samples],[r['coverage_percent'] for r in samples],label=s['label'])
    for ax,xlabel in zip(axes,('경과 시뮬레이션 시간 (초)','경과 monotonic 실제 시간 (초)')):
        ax.set(xlabel=xlabel,ylabel='고정 3364 m² 영역 관측률 (%)');ax.grid(alpha=.25);ax.legend()
    fig.suptitle('단계별 모의 탐사 — 선의 끝은 각 실행 종료, 전체 목표 달성 아님')
    fig.tight_layout();fig.savefig(output/'coverage_comparison.png',dpi=180);fig.savefig(output/'coverage_comparison.svg');plt.close(fig)
    return payload


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.root,args.output)
