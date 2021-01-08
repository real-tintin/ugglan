from dataclasses import dataclass


@dataclass
class Cuboid:
    length_x: float
    width_y: float
    height_z: float


@dataclass
class Cylinder:
    radius_xy: float
    height_z: float
