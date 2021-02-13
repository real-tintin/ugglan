import copy

import numpy as np
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D

from . import euclidean_transform as et
from .multi_body import MultiBody, Body
from .shapes import *


def duplicate_body(body, n_bodies, rotations=None, translations=None, colors=None):
    bodies = []

    if rotations is None:
        rotations = np.tile(body.rotation_rad, (n_bodies, 1))
    if translations is None:
        translations = np.tile(body.translation_m, (n_bodies, 1))
    if colors is None:
        colors = [body.color] * n_bodies

    for r, t, c in zip(rotations, translations, colors):
        new_body = copy.deepcopy(body)
        new_body.rotation_rad = r
        new_body.translation_m = t
        new_body.color = c

        bodies.append(new_body)

    return bodies


def update_inertia(m_body: MultiBody) -> None:
    """
    Computes and updates the mass and moments of
    inertia, of a multi body object.
    """
    for body in m_body.bodies:
        _compute_b_r_matrix(body)
        _compute_b_center_of_mass(body)
        _compute_b_mom_of_inertia(body)

    _compute_mb_mass(m_body)
    _compute_mb_center_of_mass(m_body)
    _compute_mb_mom_of_inertia(m_body)


def _compute_b_r_matrix(b: Body):
    b.rotation_matrix = et.rotation_matrix(*b.rotation_rad)


def _compute_b_center_of_mass(b: Body):
    b.center_of_mass = np.array([0, 0, 0])  # Note, currently only inertial translations.


def _compute_b_mom_of_inertia(b: Body):
    shape = b.shape_m

    if isinstance(shape, Cuboid):
        inertia_xx = b.mass_kg * (np.square(shape.width_y) + np.square(shape.height_z)) / 12
        inertia_yy = b.mass_kg * (np.square(shape.length_x) + np.square(shape.height_z)) / 12
        inertia_zz = b.mass_kg * (np.square(shape.length_x) + np.square(shape.width_y)) / 12

        b.mom_of_inertia = np.diag([inertia_xx, inertia_yy, inertia_zz])
    elif isinstance(shape, Cylinder):
        inertia_xx = b.mass_kg * (3 * np.square(shape.radius_xy) + np.square(shape.height_z)) / 12
        inertia_yy = inertia_xx
        inertia_zz = b.mass_kg * np.square(shape.radius_xy) / 2

        b.mom_of_inertia = np.diag([inertia_xx, inertia_yy, inertia_zz])
    else:
        raise ValueError("Invalid body shape")


def _compute_mb_mass(mb: MultiBody):
    mb.mass_kg = 0

    for b in mb.bodies:
        mb.mass_kg += b.mass_kg


def _compute_mb_center_of_mass(mb: MultiBody):
    mb.center_of_mass = np.zeros(3)

    for b in mb.bodies:
        b_cm_i_frame = b.mass_kg * (b.center_of_mass + b.translation_m)
        mb.center_of_mass += b.rotation_matrix @ np.transpose(b_cm_i_frame)


def _compute_mb_mom_of_inertia(mb: MultiBody):
    """
        I_{body}^{inertial} = R * I_{body}^{body} * R'
        d = R * (CM_{body}^{body} - p_{body}^{body}) - CM_{tot}^{inertial}

    Finally, using the parallel axis theorem:
        I_{origin}^{inertial} = I_{body}^{inertial} + m * (|r|^2 * E3 - r' * r)
    """
    mb.mom_of_inertia = np.zeros((3, 3))

    for b in mb.bodies:
        r = b.rotation_matrix

        b_moi_i_frame = r @ b.mom_of_inertia @ np.transpose(r)
        d = r @ (b.center_of_mass - b.translation_m) - mb.center_of_mass

        mb.mom_of_inertia += b_moi_i_frame + b.mass_kg * ((d @ d) * np.eye(3) - np.outer(d, d))
