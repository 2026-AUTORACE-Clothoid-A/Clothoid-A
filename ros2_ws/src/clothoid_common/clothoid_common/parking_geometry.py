"""Parking-stage dimensions from the supplied track drawing, in metres.

Origin: first near-side bay boundary centre. x follows the approach;
y points away from the bays. Only relative geometry is used at runtime.
The drawing's 500 mm road is its clear width; 20 mm paint gives 520 mm
between boundary centres. Bay clear dimensions are independent of road width.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class ParkingGeometry:
    road_width: float = .50
    line_width: float = .02
    entry_to_grid: float = .470207
    grid_to_finish: float = 2.039797

    @property
    def boundary_spacing(self):
        return self.road_width + self.line_width

    def stage_lines(self):
        """(name, endpoints, colour role); centres of the painted lines."""
        a, b, w = -self.entry_to_grid, self.grid_to_finish, self.boundary_spacing
        return (
            ('entry', ((a, 0.), (a, w)), 'white'),
            ('road_near', ((a, 0.), (b, 0.)), 'yellow'),
            ('road_far', ((a, w), (b, w)), 'yellow'),
            ('finish', ((b, 0.), (b, w)), 'yellow'),
        )


PARKING_GEOMETRY = ParkingGeometry()
