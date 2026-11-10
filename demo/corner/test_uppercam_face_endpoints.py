import math
import numpy as np
import pytest
from uppercam_lidar_geometry import fit_box_face, observe_face_endpoints


def scene(offset=0., width=.205, roll=False):
    angles=np.arange(400)*2*math.pi/400
    rotation=np.diag([1.,-1.,-1.]) if roll else np.eye(3)
    origin=np.array([.155,0.,0.]); rays=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(400)))@rotation.T
    with np.errstate(divide='ignore',invalid='ignore'):
        distance=.110/rays[:,0]
    hit=(distance>0)&(np.abs(distance*rays[:,1]-offset)<=width/2)
    ranges=np.full(400,2.);ranges[hit]=distance[hit]
    return ranges,angles,rotation,origin,hit


def run(offset=0.,width=.205,roll=False,alter=None):
    ranges,angles,rotation,origin,hit=scene(offset,width,roll)
    if alter: alter(ranges,angles,hit)
    rays=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(400)))@rotation.T
    valid=np.isfinite(ranges)&(ranges>=.1)&(ranges<=1.)
    pts=(origin+np.where(valid,ranges,0)[:,None]*rays)[valid,:2]
    prior=(.3675,0.,0.)
    fit=fit_box_face(pts,prior,(0.,0.,0.),side=.205,front_face_x=.165)
    assert fit is not None
    return observe_face_endpoints(ranges,angles,rotation,origin,fit,prior),fit


@pytest.mark.parametrize('offset,roll',[(0.,False),(.034,False),(-.034,False),(.06,False),(.034,True)])
def test_observe_known_face_and_unclipped_raw_beams(offset,roll):
    out,fit=run(offset=offset,roll=roll)
    assert out['ok'],out
    assert abs(out['box_center_map'][1]-offset)<.002
    assert out['uncertainty_m']<=.005
    if offset==.06: assert abs(fit['box_center_map'][1]-offset)>.05


def edge_after(h):
    # The face is around beam 0, so its run wraps 399 -> 0: the beam just past its end is after the low block.
    ids=np.flatnonzero(h)
    return ids[ids<len(h)//2].max()+1


def test_one_zero_at_the_edge_is_skipped():
    # The LDS-03 drops the beam straddling an edge (2026-11-10 docks): the beam past it is the evidence.
    def change(r,a,h):
        r[edge_after(h)]=0
    out,_=run(offset=.034,alter=change)
    assert out['ok'],out
    assert abs(out['box_center_map'][1]-.034)<.003


def test_two_zeros_are_not_background_evidence():
    def change(r,a,h):
        e=edge_after(h);r[e]=0;r[e+1]=0
    out,_=run(offset=.034,alter=change)
    assert not out['ok']


def test_long_wall_rejected():
    out,_=run(width=.35)
    assert not out['ok']


def test_occlusion_rejected():
    def change(r,a,h):
        ids=np.flatnonzero(h);r[ids[len(ids)//2]]=.1
    out,_=run(offset=.034,alter=change)
    assert not out['ok']


def test_front_obstacle_at_edge_rejected():
    def change(r,a,h):
        r[edge_after(h)]=.1
    out,_=run(offset=.034,alter=change)
    assert not out['ok']


def test_fov_crop_rejected():
    ranges,a,rot,origin,hit=scene(.034)
    fit=run(.034)[1]
    i=int(np.flatnonzero(~hit)[0]) + 4
    # A partial scan begins inside the visible face and ends past one edge.
    result=observe_face_endpoints(ranges[:i+1],a[:i+1],rot,origin,fit,(.3675,0.,0.))
    assert not result['ok']


@pytest.mark.parametrize('width,expected',[(.19,False),(.205,True),(.22,False)])
def test_measured_width_gate(width,expected):
    out,_=run(width=width)
    assert out['ok'] is expected


def test_two_clusters_are_not_mistaken_for_one_box():
    def change(r,a,h):
        ids=np.flatnonzero(h)
        # Two face portions, separated by five foreground rays.
        centre=ids[len(ids)//2]
        r[centre:centre+5]=.1
    out,_=run(offset=.034,alter=change)
    assert not out['ok']


def test_two_missing_interior_beams_rejected():
    def change(r,a,h):
        ids=np.flatnonzero(h)
        r[ids[len(ids)//2]:ids[len(ids)//2]+2]=0
    out,_=run(offset=.034,alter=change)
    assert not out['ok']


@pytest.mark.parametrize('angle_deg',[-4.75,4.75])
def test_rotated_face(angle_deg):
    yaw=math.radians(angle_deg);normal=np.array([-math.cos(yaw),-math.sin(yaw)])
    tangent=np.array([-normal[1],normal[0]]);face=np.array([.265,.03]);center=face-normal*.1025
    angles=np.arange(400)*2*math.pi/400
    rays=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(400)))
    origin=np.array([.155,0.,0.]);denom=rays[:,:2]@normal
    with np.errstate(divide='ignore',invalid='ignore'):
        distance=((face-origin[:2])@normal)/denom
        positions=origin[:2]+distance[:,None]*rays[:,:2]
    hit=(distance>0)&(np.abs((positions-face)@tangent)<=.1025)
    ranges=np.full(400,2.);ranges[hit]=distance[hit]
    valid=(ranges>=.1)&(ranges<=1.)
    fit=fit_box_face((origin+ranges[:,None]*rays)[valid,:2],(*center,yaw),(0.,0.,0.),side=.205,front_face_x=.165)
    assert fit is not None
    out=observe_face_endpoints(ranges,angles,np.eye(3),origin,fit,(*center,yaw),side=.205)
    assert out['ok'],out
    assert np.linalg.norm(np.array(out['box_center_map'])-center)<.002


def test_sensor_range_limit_rejects_finite_background_outside_device_range():
    ranges,angles,rotation,origin,hit=scene(.034)
    fit=run(.034)[1]
    result=observe_face_endpoints(ranges,angles,rotation,origin,fit,(.3675,0.,0.),sensor_min=.05,sensor_max=1.)
    assert not result['ok']
    assert result['edge_class']=='UNKNOWN'


def test_sensor_range_allows_finite_background_outside_operational_range():
    ranges,angles,rotation,origin,hit=scene(.034)
    fit=run(.034)[1]
    result=observe_face_endpoints(ranges,angles,rotation,origin,fit,(.3675,0.,0.),sensor_min=.05,sensor_max=12.)
    assert result['ok'],result


def test_positive_infinity_only_when_predicted_face_in_sensor_range():
    ranges,angles,rotation,origin,hit=scene(.034)
    ranges[~hit]=np.inf
    fit=run(.034)[1]
    observable=observe_face_endpoints(ranges,angles,rotation,origin,fit,(.3675,0.,0.),sensor_min=.05,sensor_max=12.)
    blind=observe_face_endpoints(ranges,angles,rotation,origin,fit,(.3675,0.,0.),sensor_min=.05,sensor_max=.12)
    assert observable['ok'],observable
    assert not blind['ok']
