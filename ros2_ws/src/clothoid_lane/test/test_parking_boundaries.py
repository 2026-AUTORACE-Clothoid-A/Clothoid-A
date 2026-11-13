import numpy as np
from clothoid_lane.parking_landmark import fit_boundary_segments


def test_observed_boundaries_fit_straight_lines_and_ignore_cross_lines():
    x=np.linspace(.4,1.,30)
    segments=fit_boundary_segments([np.column_stack((x,.02*x+.2)),
                                    np.column_stack((np.full_like(x,.6),x))])
    assert len(segments)==1
    assert np.allclose((segments[0][1]-segments[0][0])[1]/
                       (segments[0][1]-segments[0][0])[0],.02)
    assert fit_boundary_segments([[[0.,0.],[1.,float('nan')]]])==[]


def road_observations():
    import math
    angle=math.radians(6);rho=.7
    corners=[[(rho-y*np.sin(angle))/np.cos(angle),y] for y in (-.26,.26)]
    stripe=dict(angle=angle,rho=rho,corners=corners)
    x=np.linspace(.4,1.,30)
    return stripe,[np.column_stack((x,np.full_like(x,y))) for y in (-.26,.26)]


def test_two_parallel_road_lines_supply_heading_despite_skewed_stripe():
    from clothoid_lane.parking_landmark import road_heading
    stripe,sides=road_observations()
    angle,edge=road_heading(stripe,sides)
    assert abs(angle)<1e-10
    assert np.allclose(edge,stripe['corners'][0])
    assert np.allclose(road_heading(stripe,sides,side=1.)[1],stripe['corners'][1])


def test_missing_conflicting_or_wrong_width_lines_cannot_create_parking_frame():
    from clothoid_lane.parking_landmark import road_heading
    stripe,sides=road_observations()
    assert road_heading(stripe,sides[:1]) is None
    assert road_heading(stripe,None) is None
    conflict=[sides[0],sides[1]+np.column_stack((np.zeros(30),.12*(sides[1][:,0]-.7)))]
    assert road_heading(stripe,conflict) is None
    wrong_width=[sides[0],sides[1]+[0.,.2]]
    assert road_heading(stripe,wrong_width) is None
