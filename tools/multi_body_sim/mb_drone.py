import numpy as np
from PyQt5.QtGui import QColor

from multi_body_sim.multi_body import Body, MultiBody
from multi_body_sim.shapes import *
from multi_body_sim.utils import duplicate_body, ROTATE_FOUR_EQUAL_DIST

OFF_WHITE = [240, 240, 240]
DARK_RED = [139, 0, 0]

frame_top = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Top center)',
    shape_m=Cuboid(0.12, 0.12, 0.001),
    mass_kg=0.026,
    color=QColor('black')
)

frame_bottom = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Bottom center)',
    shape_m=Cuboid(0.16, 0.12, 0.001),
    mass_kg=0.080,
    trans_i_frame_m=np.array([0, 0, -0.04]),
    color=QColor('black')
)

frame_arm_1 = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Arm, part 1)',
    shape_m=Cuboid(0.20, 0.03, 0.01),
    mass_kg=0.043,
)

frame_arm_2 = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Arm, part 2)',
    shape_m=Cuboid(0.01, 0.03, 0.04),
    mass_kg=0.010,
)

frame_arm_3 = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Arm, part 3)',
    shape_m=Cuboid(0.02, 0.007, 0.055),
    mass_kg=0.005,
)

battery = Body(
    name='ZIPPY Compact 2200mAh 3S 35C Lipo Pack',
    shape_m=Cuboid(0.115, 0.034, 0.024),
    mass_kg=0.181,
    trans_i_frame_m=np.array([0, 0, -0.028]),
    color=QColor('yellow')
)

esc = Body(
    name='Afro ESC 30Amp Multi-rotor Motor Speed Controller',
    shape_m=Cuboid(0.05, 0.04, 0.005),
    mass_kg=0.0265,
    trans_i_frame_m=np.array([0.14, 0, -0.014]),
    color=QColor('green')
)

motor = Body(
    name='Turnigy L2215J-900 Brushless Motor (200 W)',
    shape_m=Cylinder(0.014, 0.03),
    mass_kg=0.076,
    trans_i_frame_m=np.array([0.23, 0, 0.015]),
    color=[51, 51, 51]
)

propeller = Body(
    name='GWS Style Slowfly Propeller 9x4.7',
    shape_m=Cuboid(0.02, 0.228, 0.001),
    mass_kg=0.007,
    trans_i_frame_m=np.array([0.23, 0, 0.03]),
    color=QColor('black')
)

mounting_surface = Body(
    name='Plexiglas mounting surface',
    shape_m=Cuboid(0.13, 0.13, 0.0017),
    mass_kg=0.040,
    trans_i_frame_m=np.array([0, 0, 0.02]),
    color=[204, 204, 204]
)

mounting_spacers = Body(
    name='Plexiglas mounting spacers',
    shape_m=Cylinder(0.003, 0.02),
    mass_kg=0.004,
    trans_i_frame_m=np.array([0.021, 0, 0.01]),
    color=[204, 204, 204]
)

i2c_breadboard = Body(
    name='I2C breadboard',
    shape_m=Cuboid(0.055, 0.04, 0.002),
    mass_kg=0.010,
    trans_i_frame_m=np.array([-0.008, 0, 0.01]),
    color=[0, 128, 0]
)

raspberry_pi = Body(
    name='Raspberry Pi Zero 2 W',
    shape_m=Cuboid(0.065, 0.03, 0.0054),
    mass_kg=0.015,
    trans_i_frame_m=np.array([-0.02, -0.04, 0.0232]),
    color=[0, 204, 0]
)

landing_gear_base = Body(
    name='Landing gear base',
    shape_m=Cuboid(0.03, 0.02, 0.004),
    mass_kg=14 / 4 * 1e-3,
    trans_i_frame_m=np.array([0.045, 0, -0.042]),
    color=[204, 204, 204]
)

landing_gear_leg = Body(
    name='Landing gear leg',
    shape_m=Cylinder(0.0025, 0.2),
    mass_kg=14 / 4 * 1e-3,
    trans_i_frame_m=np.array([0.045 + np.sin(np.pi / 8) * 0.1, 0, - 0.1 * np.cos(np.pi / 8) - 0.044]),
    rot_b_frame_rad=np.array([0, -np.pi / 8, 0]),
    color=[64, 64, 64]
)

landing_gear_foot = Body(
    name='Landing gear foot',
    shape_m=Sphere(0.008),
    mass_kg=5 / 4 * 1e-3,
    trans_i_frame_m=np.array([np.sin(np.pi / 8) * 0.2 + 0.045, 0, -np.cos(np.pi / 8) * 0.2 - 0.044]),
    color=[204, 204, 204]
)

_bodies = [
    frame_top,
    frame_bottom,
    *duplicate_body(frame_arm_1, 4, ROTATE_FOUR_EQUAL_DIST,
                    np.tile(np.array([0.15, 0, -0.006]), (4, 1)),
                    [OFF_WHITE, DARK_RED, OFF_WHITE, DARK_RED]),
    *duplicate_body(frame_arm_2, 4, ROTATE_FOUR_EQUAL_DIST,
                    np.tile(np.array([0.055, 0, -0.02]), (4, 1)),
                    [OFF_WHITE, DARK_RED, OFF_WHITE, DARK_RED]),
    *duplicate_body(frame_arm_3, 4, ROTATE_FOUR_EQUAL_DIST,
                    np.tile(np.array([0.23, 0, -0.039]), (4, 1)),
                    [OFF_WHITE, DARK_RED, OFF_WHITE, DARK_RED]),
    battery,
    *duplicate_body(esc, 4, ROTATE_FOUR_EQUAL_DIST),
    *duplicate_body(motor, 4, ROTATE_FOUR_EQUAL_DIST),
    *duplicate_body(propeller, 4, ROTATE_FOUR_EQUAL_DIST),
    mounting_surface,
    *duplicate_body(mounting_spacers, 4, ROTATE_FOUR_EQUAL_DIST),
    i2c_breadboard,
    raspberry_pi,
    *duplicate_body(landing_gear_base, 4, ROTATE_FOUR_EQUAL_DIST),
    *duplicate_body(landing_gear_leg, 4, ROTATE_FOUR_EQUAL_DIST),
    *duplicate_body(landing_gear_foot, 4, ROTATE_FOUR_EQUAL_DIST),
]

drone = MultiBody(
    name='Ugglan',
    bodies=_bodies
)
