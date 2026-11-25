"""느린 시계에서는 계산을 더 허용하고 실제 안전 기한은 그대로 지킨다."""
from concurrent.futures import Future
import tempfile
from unittest.mock import patch

from moon_mapping_demo.exploration_planning import Candidate
from test_exploration import Exploration,Field,PCFG,POSES,config,scans


class PendingPool:
    def __init__(self):self.calls=[]
    def submit(self,worker,*args):
        future=Future()
        self.calls.append((worker,args,future))
        return future


def engine(output):
    cfg=config()
    cfg['exploration']['adaptive_planning_budget']=True
    cfg['exploration']['planning_wall_budget_seconds']=8.
    cfg['risk']={'enabled':True,'safe_goal_windows':True,'multiple_escape_candidates':True,
                 'preparation_seconds':1.}
    pool=PendingPool()
    e=Exploration(cfg,Field(),PCFG,output,pool=pool)
    e.start(1.,0.,POSES)
    e.now,e.wall=4.,100.
    scans(e,e.now,POSES)
    return e,pool


def test_slow_clock_gives_worker_more_than_eight_wall_seconds():
    with tempfile.TemporaryDirectory() as out:
        e,pool=engine(out)
        e.queue([Candidate('goal',-2.,-3.)])
        assert abs(e.planning_job['clock_rate_sim_per_wall_second']-.03)<1e-9
        assert e.planning_job['wall_budget_seconds']>=150.
        assert pool.calls[-1][1][10]['planning_wall_budget_seconds']==e.planning_job['wall_budget_seconds']
        e.step(4.3,112.,POSES)
        assert e.active and e.phase=='PLANNING'
        assert e.snapshot()['planning']['wall_elapsed_seconds']==12.
        e.invalidate('검사 종료')


def test_faster_clock_discards_stale_plan_and_retries_only_after_safe_wait_check():
    with tempfile.TemporaryDirectory() as out:
        e,pool=engine(out)
        e.queue([Candidate('goal',-2.,-3.)])
        old=e.plan_id
        e.step(9.1,101.,POSES)
        assert e.active and e.phase=='WAITING_FOR_TARGETS'
        assert e.plan_id!=old and pool.calls[0][2].cancelled()
        assert e.cancel_event.is_set()


def test_expired_plan_still_stops_if_new_departure_wait_is_unsafe():
    with tempfile.TemporaryDirectory() as out:
        e,_=engine(out)
        e.queue([Candidate('goal',-2.,-3.)])
        with patch('moon_mapping_demo.exploration.stationary_safe',side_effect=[True,True,False]):
            e.step(9.1,101.,POSES)
        assert e.phase=='ERROR' and '정지 안전 구간 부족' in e.reason


def test_input_loss_still_stops_during_extended_computation():
    with tempfile.TemporaryDirectory() as out:
        e,_=engine(out)
        e.queue([Candidate('goal',-2.,-3.)])
        e.step(4.3,112.,POSES,inputs_ready=False)
        assert e.phase=='ERROR' and e.reason=='전체 odom 또는 clock 갱신 중단'


def test_shorter_safe_window_moves_departure_earlier_and_reduces_sim_budget():
    with tempfile.TemporaryDirectory() as out:
        e,_=engine(out)
        e.field=Field(danger=lambda a,b,ta,tb,r:a==b and tb>8.)
        e.queue([Candidate('goal',-2.,-3.)])
        assert e.active and e.phase=='PLANNING'
        assert e.planning_job['t0']<8.
        assert 0.<e.planning_job['sim_budget_seconds']<5.
        assert 8.<e.planning_job['wall_budget_seconds']<150.
        e.invalidate('검사 종료')


def test_budget_only_escape_failure_replans_while_stop_is_safe():
    with tempfile.TemporaryDirectory() as out:
        e,pool=engine(out)
        e.queue([],True,True)
        previous=e.plan_id
        pool.calls[-1][2].set_result({'status':'PLANNING_BUDGET','error':'계산 한도','attempts':[]})
        e.now,e.wall=4.5,115.
        e.finish_plan()
        assert e.active and e.phase=='PLANNING'
        assert e.plan_id!=previous and len(pool.calls)==2
        assert any(x['event']=='planning_budget_retry' for x in e.metrics.events)
        e.invalidate('검사 종료')


def test_budget_failure_does_not_retry_through_unsafe_wait():
    with tempfile.TemporaryDirectory() as out:
        e,pool=engine(out)
        e.queue([],True,True)
        pool.calls[-1][2].set_result({'status':'PLANNING_BUDGET','error':'계산 한도','attempts':[]})
        with patch('moon_mapping_demo.exploration.stationary_safe',return_value=False):e.finish_plan()
        assert e.phase=='ERROR'
        assert len(pool.calls)==1


def test_budget_only_normal_failure_is_not_counted_as_no_path():
    with tempfile.TemporaryDirectory() as out:
        e,pool=engine(out)
        e.queue([Candidate('goal',-2.,-3.)])
        pool.calls[-1][2].set_result({'status':'PLANNING_BUDGET','error':'계산 한도','attempts':[]})
        e.finish_plan()
        assert e.active and e.phase=='WAITING_FOR_TARGETS'
        assert e.failures==0
