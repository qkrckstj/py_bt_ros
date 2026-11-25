"""고정 분모의 탐사 시계열과 오도메트리 이동·대기 지표, 독립 그래프 출력."""
import argparse
from collections import defaultdict
import csv
import json
import math
import os
from pathlib import Path


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(temporary, path)


class Metrics:
    def __init__(self, robots, area, sim_time, wall_time, metadata=None):
        self.robots, self.area = tuple(robots), area
        self.started_sim, self.started_wall = sim_time, wall_time
        self.metadata = metadata or {}
        self.samples = []
        self.distance = dict.fromkeys(robots, 0.)
        self.wait = {r: defaultdict(float) for r in robots}
        self.previous_poses = {}
        self.odom_discontinuities = dict.fromkeys(robots, 0)
        self.replans = 0
        self.replans_by_robot = dict.fromkeys(robots, 0)
        self.rounds_completed = 0
        self.termination_reason = None
        self.events = []
        self.departures = []
        self.active_departures = {}
        self.plan_costs = []

    def pose(self, robot, stamp, xy):
        previous = self.previous_poses.get(robot)
        if previous:
            prior_stamp, prior = previous
            if stamp <= prior_stamp:
                return
            length = math.dist(xy, prior)
            # 시간 역행·위치 점프를 실제 주행 거리로 부풀리지 않는다.
            if length <= .22 * (stamp - prior_stamp) + .1:
                self.distance[robot] += length
            else:
                self.odom_discontinuities[robot] += 1
                self.previous_poses[robot] = stamp, tuple(xy)
                return
        departure=self.active_departures.get(robot)
        if (departure and departure['actual_translation_start_sim_seconds'] is None
                and stamp >= departure['t0_sim_seconds'] and
                math.dist(xy,departure['initial_xy'])>=.01):
            departure['actual_translation_start_sim_seconds']=stamp
        self.previous_poses[robot] = stamp, tuple(xy)

    def register_departures(self,session,plan_id,round_id,batch,poses,tasks):
        schedule=batch.get('departure',{})
        self.plan_costs.append({'session':session,'plan_id':plan_id,'round_id':round_id,
            'assignment': {'columns':list(batch['assignment'].columns), 'total_cost':batch['assignment'].total_cost},
            'costs':batch['costs'],'robot_ids':list(batch['assigned']),
            'goal_ids_by_robot':{r:g.id for r,g in batch['assigned'].items()},
            't0_sim_seconds':batch['t0'],'breakdown':batch.get('cost_breakdown'),
            'time_model':batch.get('time_model','space_time_move_wait_post_rotation'),
            'execution_hold_seconds':batch.get('execution_hold_seconds'),
            'departure_policy':schedule.get('policy','simultaneous')})
        for robot in batch['assigned']:
            record={'session':session,'plan_id':plan_id,'round_id':round_id,'robot':robot,
                'task_id':tasks[robot],'role':'follow' if robot in schedule.get('follow_robots',[]) else 'lead',
                'policy':schedule.get('policy','simultaneous'),'t0_sim_seconds':batch['t0'],
                'scheduled_release_sim_seconds':schedule.get('release_times',{}).get(robot,batch['t0']),
                'planned_translation_start_sim_seconds':schedule.get('planned_translation_start_times',{}).get(robot),
                'actual_translation_start_sim_seconds':None,'first_linear_command_sim_seconds':None,
                'initial_xy':list(poses[robot][:2]),'ended_sim_seconds':None,'ended_reason':None}
            self.departures.append(record)
            self.active_departures[robot]=record

    def retire_departures(self,plan_id,now,reason):
        for robot,record in list(self.active_departures.items()):
            if record['plan_id']==plan_id:
                record['ended_sim_seconds'],record['ended_reason']=now,reason
                self.active_departures.pop(robot)

    def note_command(self,robot,now,linear):
        record=self.active_departures.get(robot)
        if record and linear>1e-4 and record['first_linear_command_sim_seconds'] is None:
            record['first_linear_command_sim_seconds']=now

    def waiting(self, dt, reasons):
        if dt <= 0:
            return
        for robot, reason in reasons.items():
            if reason:
                self.wait[robot][reason] += dt

    def replan(self, sim_time, reason, affected):
        self.replans += 1
        for robot in affected:
            self.replans_by_robot[robot] += 1
        self.events.append({'event': 'replan', 'sim_time': sim_time, 'reason': reason,
                            'affected_robots': list(affected)})

    def sample(self, observation, sim_time, wall_time, phase, force=False, interval=1.):
        if self.samples and not force and sim_time - self.samples[-1]['sim_time'] < interval:
            return
        state = observation.snapshot()
        sample = {'sim_time': sim_time, 'elapsed_sim_seconds': max(0., sim_time - self.started_sim),
                  'elapsed_wall_seconds': max(0., wall_time - self.started_wall), 'phase': phase,
                  'observed_area_m2': state['observed_area_m2'],
                  'coverage_percent': 100. * state['coverage_fraction'],
                  'denominator_m2': state['denominator_m2'], 'replans': self.replans,
                  'rounds_completed': self.rounds_completed}
        latest=self.plan_costs[-1] if self.plan_costs else {}
        sample['last_planned_plan_id']=latest.get('plan_id')
        for key in ('hungarian_total_ticks','coordinated_total_ticks','initial_departure_wait_ticks',
                    'staging_wait_ticks','turn_ticks','independent_assignment_turn_ticks','additional_turn_ticks',
                    'route_and_other_wait_delta_ticks','additional_total_ticks'):
            sample[key]=(latest.get('breakdown') or {}).get(key)
        for r in self.robots:
            sample[r + '.new_observed_area_m2'] = state['new_area_m2_by_robot'][r]
            sample[r + '.odom_distance_m'] = self.distance[r]
            sample[r + '.replans'] = self.replans_by_robot[r]
            sample[r + '.waiting_sim_seconds'] = sum(v for k, v in self.wait[r].items()
                                                    if k not in ('observing', 'turning'))
            sample[r + '.observing_sim_seconds'] = self.wait[r].get('observing', 0.)
            sample[r + '.turning_sim_seconds'] = self.wait[r].get('turning', 0.)
            sample[r + '.departure_wait_sim_seconds'] = self.wait[r].get('departure_wait',0.)
            departure=self.active_departures.get(r,{})
            sample[r+'.departure_plan_id']=departure.get('plan_id')
            sample[r+'.departure_task_id']=departure.get('task_id')
            sample[r+'.departure_role']=departure.get('role')
            for key in ('scheduled_release_sim_seconds','planned_translation_start_sim_seconds',
                        'actual_translation_start_sim_seconds','first_linear_command_sim_seconds'):
                sample[r+'.'+key]=departure.get(key)
        self.samples.append(sample)

    def payload(self, observation):
        return {'schema_version': 'limo-exploration-metrics/v1',
                'time_origin': 'ROS /clock = Webots absolute simulation seconds',
                'started_sim_seconds': self.started_sim, 'evaluation': vars(self.area),
                'fixed_denominator_m2': self.area.denominator_m2,
                'metadata': self.metadata, 'observation': observation.snapshot(),
                'termination_reason': self.termination_reason,
                'distance_source': 'timestamped wheel/IMU odometry; not simulator ground truth',
                'wait_definition': 'planning/schedule/idle/safety hold; observing and turning separate',
                'distance_m_by_robot': self.distance,
                'waiting_sim_seconds_by_reason': {r: dict(v) for r, v in self.wait.items()},
                'odom_discontinuities': self.odom_discontinuities,
                'replans': self.replans, 'replans_by_robot': self.replans_by_robot,
                'rounds_completed': self.rounds_completed, 'events': self.events,
                'first_hungarian_assignment':self.plan_costs[0] if self.plan_costs else None,
                'plan_costs':self.plan_costs,'departures':self.departures,
                'actual_departure_definition':'해당 plan 시작 위치에서 유효 stamped odom 변위가 0.01m 이상인 최초 시각; 명령 시각 별도 기록',
                'samples': self.samples}

    def save(self, output, observation, plot=False):
        output = Path(output)
        payload = self.payload(observation)
        atomic_json(output / 'metrics.json', payload)
        if self.samples:
            path = output / 'metrics.csv'
            with path.with_suffix('.csv.tmp').open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0]))
                writer.writeheader()
                writer.writerows(self.samples)
            os.replace(path.with_suffix('.csv.tmp'), path)
        if plot and self.samples:
            plot_metrics(payload, output)


def plot_metrics(payload, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    fonts = {f.name for f in font_manager.fontManager.ttflist}
    font = next((n for n in ('Noto Sans CJK JP', 'NanumGothic', 'UnDotum') if n in fonts), None)
    korean = font is not None
    labels = (('경과 시뮬레이션 시간 (초)', '실제 경과 시간 (초)', '탐사율 (%)', '최초 신규 관측 면적 (m²)')
              if korean else ('Elapsed simulation time (s)', 'Elapsed wall time (s)',
                              'Observed coverage (%)', 'First-fused new area (m²)'))
    samples = payload['samples']
    with plt.rc_context({'font.family': font or 'DejaVu Sans', 'axes.unicode_minus': False}):
        fig, axes = plt.subplots(3, 1, figsize=(9, 10), constrained_layout=True)
        axes[0].plot([r['elapsed_sim_seconds'] for r in samples], [r['coverage_percent'] for r in samples])
        axes[1].plot([r['elapsed_wall_seconds'] for r in samples], [r['coverage_percent'] for r in samples])
        for axis, xlabel in zip(axes[:2], labels[:2]):
            axis.set(xlabel=xlabel, ylabel=labels[2], ylim=(0, 100))
            axis.grid(alpha=.3)
        for robot in payload['distance_m_by_robot']:
            axes[2].plot([r['elapsed_sim_seconds'] for r in samples],
                         [r[robot + '.new_observed_area_m2'] for r in samples], label=robot)
        axes[2].set(xlabel=labels[0], ylabel=labels[3])
        axes[2].legend()
        axes[2].grid(alpha=.3)
        source = ('모의 센서 · 이상 기구학' if payload['metadata'].get('synthetic_sensor') else '거리 센서 관측') if korean else (
            'Synthetic sensor / ideal kinematics' if payload['metadata'].get('synthetic_sensor') else 'Range sensor observations')
        fig.suptitle(f"{source}\nROI = {payload['fixed_denominator_m2']:.2f} m²; {payload['termination_reason'] or 'RUNNING'}")
        output = Path(output)
        fig.savefig(output / 'coverage.png', dpi=150)
        fig.savefig(output / 'coverage.svg')
        plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metrics', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.metrics.parent
    output.mkdir(parents=True, exist_ok=True)
    plot_metrics(json.loads(args.metrics.read_text()), output)


if __name__ == '__main__':
    main()
