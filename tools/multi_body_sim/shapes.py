from dataclasses import dataclass
from typing import Union


@dataclass
class Cuboid:
    length_x: float
    width_y: float
    height_z: float


@dataclass
class Cylinder:
    radius_xy: float
    height_z: float


@dataclass
class Sphere:
    radius: float


Shapes = Union[Cuboid, Cylinder, Sphere]
