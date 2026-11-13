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
