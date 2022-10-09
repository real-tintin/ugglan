import numpy as np
import pytest
from copy import deepcopy

import ugglan_tools.multi_body_sim.utils as mb_utils
from ugglan_tools.multi_body_sim.multi_body import Body, MultiBody
from ugglan_tools.multi_body_sim.shapes import Cuboid, Sphere
from unit_bodies import UNIT_CYLINDER, UNIT_CUBOID, UNIT_SPHERE

MAGIC_BOX = MultiBody(
    name='magic_cuboid',
    bodies=[Body(name='unit_cuboid',
                 shape_m=Cuboid(length_x=1, width_y=2, height_z=3),
                 mass_kg=0.123)]
)
mb_utils.update_inertia(MAGIC_BOX)


@pytest.mark.parametrize("n_bodies", [1, 10])
@pytest.mark.parametrize("overwrite", [False, True])
def test_duplicate_body(n_bodies, overwrite):
    body = Body(name="not_human", shape_m=Cuboid(1, 2, 3), mass_kg=70)

    if overwrite:
        rotations = np.tile(np.array([0, np.pi, 0]), (n_bodies, 1))
        translations = np.tile(np.array([0, np.pi, 0]), (n_bodies, 1))
        colors = "red" * n_bodies

        bodies = mb_utils.duplicate_body(body, n_bodies, rotations,
                                         translations, colors)
    else:
        bodies = mb_utils.duplicate_body(body, n_bodies)

    assert len(bodies) == n_bodies


class TestInertia:
    @pytest.mark.parametrize("unit_mb", [UNIT_CYLINDER, UNIT_CUBOID, UNIT_SPHERE])
    def test_unit_mb(self, unit_mb):
        mb_utils.update_inertia(unit_mb)

        np.testing.assert_array_almost_equal(unit_mb.com_m, np.zeros(3))
        np.testing.assert_array_almost_equal(unit_mb.moi_kg2, np.eye(3))

    @pytest.mark.parametrize("rot, diag_ind_exp", [
        ([np.pi / 2, 0, 0], [0, 2, 1]),
        ([0, -np.pi / 2, 0], [2, 1, 0]),
        ([0, 0, np.pi / 2], [1, 0, 2]),
    ])
    def test_rot_b(self, rot, diag_ind_exp):
        mb = deepcopy(MAGIC_BOX)

        mb.bodies[0].rot_b_frame_rad = rot

        mb_utils.update_inertia(mb)

        np.testing.assert_array_almost_equal(mb.com_m, np.zeros(3))
        np.testing.assert_array_almost_equal(np.diag(mb.moi_kg2), np.diag(MAGIC_BOX.moi_kg2)[diag_ind_exp])

    @pytest.mark.parametrize("rot", [
        ([np.pi / 4, 0]),
        ([0, -np.pi / 2, ]),
        ([0, 0, -np.pi / 8]),
    ])
    def test_around_the_world(self, rot):
        mb = deepcopy(MAGIC_BOX)

        mb.bodies[0].rot_b_frame_rad = np.array(rot)
        mb.bodies[0].rot_i_frame_rad = -np.array(rot)

        mb_utils.update_inertia(mb)

        np.testing.assert_array_almost_equal(mb.com_m, MAGIC_BOX.com_m)
        np.testing.assert_array_almost_equal(mb.moi_kg2, MAGIC_BOX.moi_kg2)

    def test_multi_tf(self):
        mb = deepcopy(UNIT_CUBOID)

        mb.bodies[0].rot_b_frame_rad = np.array([0, np.pi / 4, 0])
        mb.bodies[0].trans_i_frame_m = np.array([0, 0, 10])
        mb.bodies[0].rot_i_frame_rad = np.array([np.pi / 2, 0, 0])

        mb_utils.update_inertia(mb)

        np.testing.assert_almost_equal(mb.com_m, [0, -10, 0])

    def test_multi_body(self):
        small = Body(name='small', shape_m=Sphere(1), mass_kg=1e-8, trans_i_frame_m=np.array([0.1, 0, 0]))
        large = Body(name='large', shape_m=Sphere(1), mass_kg=1.00, trans_i_frame_m=np.array([-0.1, 0, 0]))

        mb = MultiBody(name='multi_body',
                       bodies=[small, large])

        mb_utils.update_inertia(mb)

        np.testing.assert_array_almost_equal(mb.com_m, [-0.1, 0, 0])
        assert_is_diag(mb.moi_kg2)


def assert_is_diag(m):
    np.testing.assert_array_almost_equal(m - np.diag(np.diagonal(m)), np.zeros(m.shape))
