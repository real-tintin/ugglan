import numpy as np

from ugglan_tools.multi_body_sim.multi_body import Body, MultiBody
from ugglan_tools.multi_body_sim.shapes import Cuboid, Cylinder, Sphere

UNIT_CYLINDER = MultiBody(
    name='one_unit_cylinder',
    bodies=[Body(name='unit_cylinder',
                 shape_m=Cylinder(radius_xy=np.sqrt(2), height_z=np.sqrt(6)),
                 mass_kg=1)]
)

UNIT_CUBOID = MultiBody(
    name='one_unit_cuboid',
    bodies=[Body(name='unit_cuboid',
                 shape_m=Cuboid(length_x=np.sqrt(6), width_y=np.sqrt(6), height_z=np.sqrt(6)),
                 mass_kg=1)]
)

UNIT_SPHERE = MultiBody(
    name='one_unit_sphere',
    bodies=[Body(name='unit_sphere',
                 shape_m=Sphere(radius=np.sqrt(5 / 2)),
                 mass_kg=1)]
)
