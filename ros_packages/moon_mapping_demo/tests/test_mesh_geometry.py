"""삼각형 형상·원형 여유·같은 시간의 그림자 검사. 구형 사각형 판정과 구분한다."""
import math
import numpy as np

from moon_mapping_demo.mesh_geometry import capsule_hits_triangles,clip_above,MeshShadow


def test_triangle_empty_corner_is_available_but_real_body_contact_is_not():
    triangle=np.array([[[0.,0.],[2.,0.],[0.,2.]]])
    assert not capsule_hits_triangles((1.7,1.7),(1.7,1.7),triangle,.1)
    assert not capsule_hits_triangles((1.1,1.1),(1.1,1.1),triangle,.1)
    assert capsule_hits_triangles((1.1,1.1),(1.1,1.1),triangle,.15)
    assert capsule_hits_triangles((-1.,.5),(2.,.5),triangle,.01)


def test_degenerate_triangle_does_not_block_entire_plane_and_edge_distance_is_round():
    triangle=np.array([[[0.,0.],[1.,0.],[2.,0.]]])
    assert not capsule_hits_triangles((50.,0.),(50.,0.),triangle,.1)
    assert capsule_hits_triangles((1.,.05),(1.,.05),triangle,.1)
    assert not capsule_hits_triangles((2.08,.08),(2.08,.08),triangle,.1)
    assert not capsule_hits_triangles((3.,0.),(4.,0.),triangle,0.)


def test_clipping_keeps_only_occluders_above_receiver_plane():
    triangles=np.array([[[0.,0.,-1.],[2.,0.,1.],[0.,2.,1.]],
                        [[0.,0.,-2.],[1.,0.,-2.],[0.,1.,-2.]]])
    clipped=clip_above(triangles,0.)
    assert clipped.shape==(2,3,3)
    assert np.all(clipped[:,:,2]>=0.)
    assert np.any(clipped[:,:,2]==0.)


def shadow(step=.025,period=240.):
    triangle=np.array([[[0.,0.,1.],[.1,0.,1.],[0.,.1,1.]]])
    return MeshShadow(triangle,45.,0.,period,step,plane_height=0.)


def test_motion_and_shadow_are_synchronized_instead_of_union_of_entire_interval():
    field=shadow()
    # 같은 시각에는 그림자 앞쪽으로 떨어져 이동한다. 전체 구간 AABB와는 겹친다.
    assert not field.blocked((.5,-1.),(-.5,0.),0.,60.,.05)
    for t in np.linspace(0.,60.,121):
        fraction=t/60.
        p=(.5-fraction,-1.+fraction)
        assert not field.blocked(p,p,float(t),float(t),.05)


def test_shadow_sweeping_body_between_clear_endpoints_is_detected():
    field=shadow()
    p=(-.97,.03)
    assert not field.blocked(p,p,0.,0.,.01)
    assert not field.blocked(p,p,120.,120.,.01)
    assert field.blocked(p,p,60.,60.,.01)
    assert field.blocked(p,p,0.,120.,.01)


def test_time_bucket_search_matches_unfiltered_triangles_across_period_boundary():
    rng=np.random.default_rng(418)
    triangles=rng.uniform(-2.,2.,(40,3,3))
    triangles[:,:,2]=rng.uniform(.02,2.,(40,3))
    field=MeshShadow(triangles,30.,270.,31.,.05)
    for i in range(100):
        a=rng.uniform(-4.,4.,2)
        b=a+rng.uniform(-.5,.5,2)
        ta=30.99+i*.317
        tb=ta+(0.,.25,3.,20.)[i%4]
        actual=field.blocked(a,b,ta,tb,.22)
        expected=field._interval(a,b,ta,tb,.22,np.arange(len(field.triangles)))
        assert actual==expected


def test_dense_point_ray_hits_are_never_missed_by_time_interval_guard():
    field=shadow(step=.05,period=4.)
    rng=np.random.default_rng(93)
    for _ in range(80):
        ta=float(rng.uniform(0.,8.));tb=ta+float(rng.uniform(.01,3.))
        a=rng.uniform(-1.2,1.2,2);b=a+rng.uniform(-.4,.4,2)
        collided=False
        for t in np.linspace(ta,tb,101):
            p=a+(b-a)*(t-ta)/(tb-ta)
            angle=2*math.pi*t/4.
            # z=1 삼각형에 대한 +태양 광선 교차의 직접 식. 투영 함수와 독립이다.
            q=p+np.array([math.sin(angle),math.cos(angle)])
            if q[0]>=0. and q[1]>=0. and q.sum()<=.1:collided=True;break
        if collided:assert field.blocked(a,b,ta,tb,0.)
