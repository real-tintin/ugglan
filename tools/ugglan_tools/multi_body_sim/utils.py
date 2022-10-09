import copy

import numpy as np
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D

import ugglan_tools.multi_body_sim.euclidean_transform as et
from ugglan_tools.multi_body_sim.multi_body import MultiBody, Body
from ugglan_tools.multi_body_sim.shapes import Cuboid, Cylinder, Sphere

ROTATE_FOUR_EQUAL_DIST = np.array([[0, 0, np.pi / 4],
                                   [0, 0, 3 * np.pi / 4],
                                   [0, 0, -np.pi / 4],
                                   [0, 0, -3 * np.pi / 4]])


def duplicate_body(body, n_bodies, rot_i=None, trans_i=None, colors=None):
    bodies = []

    if rot_i is None:
        rot_i = np.tile(body.rot_i_frame_rad, (n_bodies, 1))
    if trans_i is None:
        trans_i = np.tile(body.trans_i_frame_m, (n_bodies, 1))
    if colors is None:
        colors = [body.color] * n_bodies

    for r_i, t_i, c in zip(rot_i, trans_i, colors):
        new_body = copy.deepcopy(body)
        new_body.rot_i_frame_rad = r_i
        new_body.trans_i_frame_m = t_i
        new_body.color = c

        bodies.append(new_body)

    return bodies


def update_inertia(m_body: MultiBody) -> None:
    """
    Computes and updates the mass and moments of
    inertia, of a multi body object.
    """
    for body in m_body.bodies:
        _compute_com(body)
        _compute_b_moi(body)

    _compute_mb_mass(m_body)
    _compute_mb_com(m_body)
    _compute_mb_moi(m_body)


def _compute_com(b: Body):
    b.com_b_frame_m = np.array([.0, .0, .0])
    b.com_i_frame_m = et.rotate(et.translate(b.com_b_frame_m, *b.trans_i_frame_m), *b.rot_i_frame_rad)


def _compute_b_moi(b: Body):
    shape = b.shape_m

    if isinstance(shape, Cuboid):
        moi_xx = b.mass_kg * (np.square(shape.width_y) + np.square(shape.height_z)) / 12
        moi_yy = b.mass_kg * (np.square(shape.length_x) + np.square(shape.height_z)) / 12
        moi_zz = b.mass_kg * (np.square(shape.length_x) + np.square(shape.width_y)) / 12

    elif isinstance(shape, Cylinder):
        moi_xx = b.mass_kg * (3 * np.square(shape.radius_xy) + np.square(shape.height_z)) / 12
        moi_yy = moi_xx
        moi_zz = b.mass_kg * np.square(shape.radius_xy) / 2

    elif isinstance(shape, Sphere):
        moi_xx = 2 / 5 * b.mass_kg * shape.radius ** 2
        moi_yy = moi_xx
        moi_zz = moi_xx

    else:
        raise ValueError("Invalid body shape")

    b.moi_b_frame_kg2 = _tf_moi(
        mass=b.mass_kg,
        moi_b=np.diag([moi_xx, moi_yy, moi_zz]),
        com_b=b.com_b_frame_m,
        rot=b.rot_b_frame_rad
    )


def _compute_mb_mass(mb: MultiBody):
    mb.mass_kg = 0

    for b in mb.bodies:
        mb.mass_kg += b.mass_kg


def _compute_mb_com(mb: MultiBody):
    mb.com_m = np.array([.0, .0, .0])

    for b in mb.bodies:
        mb.com_m += b.mass_kg * b.com_i_frame_m


def _compute_mb_moi(mb: MultiBody):
    mb.moi_kg2 = np.zeros((3, 3))

    for b in mb.bodies:
        b.moi_i_frame_kg2 = _tf_moi(
            mass=b.mass_kg,
            moi_b=b.moi_b_frame_kg2,
            com_b=b.com_i_frame_m,
            com_i=mb.com_m,
            rot=b.rot_i_frame_rad
        )

        mb.moi_kg2 += b.moi_i_frame_kg2


def _tf_moi(mass, moi_b, rot, com_b, com_i=np.array([.0, .0, .0])):
    """
    First rotate:
        I_{body}^{inertial} = R * I_{body}^{body} * R'

    then use the parallel axis theorem:
        d = CM_{inertial} - CM_{body}
        I_{o}^{inertial} = I_{body}^{inertial} + m * (d * d' * E3 - d' * d)
    """
    r = et.rotation_matrix(*rot)
    moi_b_rot = r @ moi_b @ np.transpose(r)

    d = com_i - com_b
    moi_i = moi_b_rot + mass * ((d @ d) * np.eye(3) - np.outer(d, d))

    return moi_i
