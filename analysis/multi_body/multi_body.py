from dataclasses import dataclass
from typing import Union, List

import numpy as np

from plot_3d_shape import Cuboid, Cylinder


@dataclass()
class Body:
    name: str

    shape_m: Union[Cuboid, Cylinder]
    mass_kg: float
    rotation_rad: np.array = np.array([0, 0, 0])
    translation_m: np.array = np.array([0, 0, 0])

    color: Union[str, List[float]] = None

    center_of_mass: np.array = None
    mom_of_inertia: np.array = None

    rotation_matrix: np.array = None


@dataclass()
class MultiBody:
    name: str
    bodies: List[Body]

    mass_kg: float = None
    center_of_mass: np.array = None
    mom_of_inertia: np.array = None

