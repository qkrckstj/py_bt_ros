#!/usr/bin/env python3
"""저장된 실패를 원본 WBT 삼각형/투영 그림자와 보수적 검사 모델로 대조한다.

고밀도 시각화 표본은 진단 자료이며 연속 안전 검사를 대신하지 않는다.
"""
import argparse
import json
import math
from pathlib import Path
import re
import numpy as np

from moon_mapping_demo.experiment import load_experiment,make_field
from moon_mapping_demo.metrics import atomic_json
from moon_mapping_demo.terrain import _blocks,_rotation,_numbers,sun_direction,_segment_hits_boxes


def raw_triangles(world):
    triangles=[]
    for name,kind,block in _blocks(world.read_text()):
        point=re.search(r'\bpoint\s*\[([^]]*)\]',block,re.S)
        indices=re.search(r'\bcoordIndex\s*\[([^]]*)\]',block,re.S)
        if kind!='Solid' or not point or not indices:continue
        tr=re.search(r'^\s*translation\s+([^\n]+)',block)
        rot=re.search(r'^\s*rotation\s+([^\n]+)',block)
        xyz=_numbers(point.group(1)).reshape(-1,3)
        xyz=xyz@_rotation(_numbers(rot.group(1)) if rot else (0,0,1,0)).T
        xyz+=_numbers(tr.group(1)) if tr else np.zeros(3)
        face=[]
        for i in _numbers(indices.group(1)).astype(int):
            if i==-1:
                triangles.extend(xyz[[face[0],face[k],face[k+1]]] for k in range(1,len(face)-1))
                face=[]
            else:face.append(i)
    return np.asarray(triangles)


def run(trace,config,output):
    if output.exists():raise FileExistsError('새 진단 결과 폴더를 지정하세요.')
    output.mkdir(parents=True)
    cfg,env,pcfg,share=load_experiment(config);field=make_field(cfg,env,pcfg,share)
    state=json.loads(trace.read_text());ta=state['sim_time']
    a=tuple(state['poses']['robot3'][:2])
    next_point=next(p for p in state['paths']['robot3']['points'] if p['t']>ta)
    b=(next_point['x'],next_point['y']);tb=next_point['t']
    # 현행 oracle은 구간 그림자 AABB와 swept 몸체를 교차한다.
    # 아래 동시 시각 표본은 구간 감쌈의 보수성 진단에만 사용한다.
    samples=[]
    for t in np.linspace(ta,tb,301):
        f=(t-ta)/(tb-ta);xy=(a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]))
        if field.blocked_segment(xy,xy,float(t),float(t),pcfg.robot_radius):samples.append(float(t))
    mesh=raw_triangles(share/'worlds'/env['world'])
    positive=mesh[np.any(mesh[:,:,2]>.001,axis=1)]
    direction=np.asarray(sun_direction(ta,field.elevation,field.initial_azimuth,field.period))
    projected=positive[:,:,:2]-positive[:,:,2,None]*direction[:2]/direction[2]
    info={'source_trace':str(trace),'absolute_sim_interval':[ta,tb],'robot':'robot3',
        'actual_start_xy':a,'planned_segment_end_xy':b,'body_radius_m':pcfg.robot_radius,
        'static_forbidden':field.static_blocked_segment(a,b,pcfg.robot_radius),
        'interval_shadow_forbidden':_segment_hits_boxes(a,b,field.shadow_boxes(ta,tb),pcfg.robot_radius),
        'instantaneous_model_samples':301,'sampled_forbidden_times':samples,
        'current_body_forbidden':field.blocked_segment(a,a,ta,ta,pcfg.robot_radius),
        'stopped_through_next_t0_forbidden':field.blocked_segment(a,a,ta,tb,pcfg.robot_radius),
        'interpretation':('구간 감쌈 검사는 금지이며 같은 시각의 몸체 표본은 금지를 발견하지 못함. '
            '공간·시간 감쌈의 보수성이 관여할 수 있으나 표본 검사로 안전을 증명하지 않음. '
            '원본 WBT 삼각형의 지면 투영 그림자는 대조용이며 실행 oracle을 바꾸지 않음.')
            if not samples else '같은 시각의 몸체 표본에서도 현행 그림자 모델 금지를 확인함. 원본 삼각형 그림자와 대조 필요.',
        'safety_model_modified':False,'webots_visual_confirmation':False}
    atomic_json(output/'geometry_diagnosis.json',info)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle,Circle
    fig,axes=plt.subplots(1,2,figsize=(12,5),sharex=True,sharey=True)
    for ax in axes:
        ax.add_collection(PolyCollection(positive[:,:,:2],facecolor='gray',edgecolor='gray',alpha=.25))
        ax.plot([a[0],b[0]],[a[1],b[1]],'b.-',label='robot3 남은 이동 구간')
        ax.add_patch(Circle(a,.26,fill=False,color='b',label='몸체 반경 0.26 m'))
        ax.set(xlim=(-24,-20),ylim=(-26.5,-22.5),xlabel='x (m)',ylabel='y (m)')
        ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0].add_collection(PolyCollection(projected,facecolor='orange',edgecolor='none',alpha=.3))
    axes[0].set_title(f'원본 WBT 삼각형·투영 그림자 (sim {ta:.1f}초)')
    for box in field.shadow_boxes(ta,tb):
        if box[2]<-24 or box[0]>-20 or box[3]<-26.5 or box[1]>-22.5:continue
        axes[1].add_patch(Rectangle(box[:2],box[2]-box[0],box[3]-box[1],color='orange',alpha=.16))
    axes[1].set_title(f'현행 기둥·구간 그림자 감쌈 (sim {ta:.1f}–{tb:.1f}초)')
    axes[0].legend(loc='lower left')
    fig.tight_layout();fig.savefig(output/'failure_geometry.png',dpi=180);fig.savefig(output/'failure_geometry.svg');plt.close(fig)
    return info


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config/exploration_wide.yaml')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(json.dumps(run(args.trace,args.config,args.output),ensure_ascii=False,indent=2))
