from dataclasses import dataclass
from typing import Union, List

import numpy as np
from PyQt5.QtGui import QColor

from multi_body_sim.shapes import Shapes


@dataclass()
class Body:
    name: str

    shape_m: Shapes
    mass_kg: float
    color: Union[QColor, List[int]] = QColor('black')

    # Order of transformation: R_i * T_i * R_b
    rot_b_frame_rad: np.array = np.array([.0, .0, .0])
    trans_i_frame_m: np.array = np.array([.0, .0, .0])
    rot_i_frame_rad: np.array = np.array([.0, .0, .0])

    com_b_frame_m: np.array = None
    com_i_frame_m: np.array = None

    moi_b_frame_kg2: np.array = None
    moi_i_frame_kg2: np.array = None


@dataclass()
class MultiBody:
    name: str
    bodies: List[Body]

    mass_kg: float = None
    com_m: np.array = None
    moi_kg2: np.array = None
