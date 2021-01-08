import numpy as np
import pytest

import multi_body_utils
from multi_body import Body, MultiBody
from plot_shape import Cuboid, Cylinder

UNIT_MB_CYLINDER = MultiBody(
    name='one_unit_cylinder',
    bodies=[Body(name='unit_cylinder',
                 shape_m=Cylinder(radius_xy=np.sqrt(2), height_z=np.sqrt(6)),
                 mass_kg=1)]
)

UNIT_MB_CUBOID = MultiBody(
    name='one_unit_cuboid',
    bodies=[Body(name='unit_cuboid',
                 shape_m=Cuboid(length_x=np.sqrt(6), width_y=np.sqrt(6), height_z=np.sqrt(6)),
                 mass_kg=1)]
)


@pytest.mark.parametrize("n_bodies", [1, 10])
@pytest.mark.parametrize("overwrite", [False, True])
def test_duplicate_body(n_bodies, overwrite):
    body = Body(name="not_human", shape_m=Cuboid(1, 2, 3), mass_kg=70)

    if overwrite:
        rotations = np.tile(np.array([0, np.pi, 0]), (n_bodies, 1))
        translations = np.tile(np.array([0, np.pi, 0]), (n_bodies, 1))
        colors = "red" * n_bodies

        bodies = multi_body_utils.duplicate_body(body, n_bodies, rotations,
                                                 translations, colors)
    else:
        bodies = multi_body_utils.duplicate_body(body, n_bodies)

    assert len(bodies) == n_bodies


@pytest.mark.parametrize("unit_mb", [UNIT_MB_CYLINDER, UNIT_MB_CUBOID])
def test_inertia(unit_mb):
    multi_body_utils.update_inertia(unit_mb)

    assert unit_mb.mass_kg == 1
    np.testing.assert_array_almost_equal(unit_mb.center_of_mass, np.zeros(3))
    np.testing.assert_array_almost_equal(unit_mb.mom_of_inertia, np.eye(3))

