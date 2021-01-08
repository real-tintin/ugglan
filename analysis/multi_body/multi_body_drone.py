import numpy as np

from multi_body import Body, MultiBody
from multi_body_utils import duplicate_body
from shapes import *

ROTATE_FOUR_EQUAL_SPACE = np.array([[0, 0, np.pi / 4],
                                    [0, 0, 3 * np.pi / 4],
                                    [0, 0, -np.pi / 4],
                                    [0, 0, -3 * np.pi / 4]])

frame_top = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Top center)',
    shape_m=Cuboid(0.12, 0.12, 0.001),
    mass_kg=0.026,
    color='black'
)

frame_bottom = Body(
    name='Q450 V3 Glass Fiber Quadcopter Frame 450mm (Bottom center)',
    shape_m=Cuboid(0.16, 0.12, 0.001),
    mass_kg=0.080,
    translation_m=np.array([0, 0, -0.04]),
    color='black'
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
    translation_m=np.array([0, 0, -0.028]),
    color='yellow'
)

esc = Body(
    name='Afro ESC 30Amp Multi-rotor Motor Speed Controller',
    shape_m=Cuboid(0.05, 0.04, 0.005),
    mass_kg=0.0265,
    translation_m=np.array([0.14, 0, -0.014]),
    color='green'
)

motor = Body(
    name='Turnigy L2215J-900 Brushless Motor (200 W)',
    shape_m=Cylinder(0.014, 0.03),
    mass_kg=0.076,
    translation_m=np.array([0.23, 0, 0.015]),
    color=[0.2, 0.2, 0.2]
)

propeller = Body(
    name='GWS Style Slowfly Propeller 9x4.7',
    shape_m=Cuboid(0.02, 0.228, 0.001),
    mass_kg=0.007,
    translation_m=np.array([0.23, 0, 0.03]),
    color='black'
)

mounting_surface = Body(
    name='Plexiglas mounting surface',
    shape_m=Cuboid(0.13, 0.13, 0.0017),
    mass_kg=0.040,
    translation_m=np.array([0, 0, 0.02]),
    color=[0.8, 0.8, 0.8]
)

mounting_spacers = Body(
    name='Plexiglas mounting spacers',
    shape_m=Cylinder(0.003, 0.02),
    mass_kg=0.004,
    translation_m=np.array([0.021, 0, 0.01]),
    color=[0.8, 0.8, 0.8]
)

i2c_breadboard = Body(
    name='I2C breadboard',
    shape_m=Cuboid(0.055, 0.04, 0.002),
    mass_kg=0.010,
    translation_m=np.array([-0.008, 0, 0.01]),
    color=[0, 0.5, 0]
)

raspberry_pi = Body(
    name='Raspberry Pi Zero',
    shape_m=Cuboid(0.065, 0.03, 0.0054),
    mass_kg=0.015,
    translation_m=np.array([-0.02, -0.04, 0.0232]),
    color=[0, 0.8, 0]
)

_bodies = [
    frame_top,
    frame_bottom,
    *duplicate_body(frame_arm_1, 4, ROTATE_FOUR_EQUAL_SPACE,
                    np.tile(np.array([0.15, 0, -0.006]), (4, 1)),
                    ['white', 'red', 'white', 'red']),
    *duplicate_body(frame_arm_2, 4, ROTATE_FOUR_EQUAL_SPACE,
                    np.tile(np.array([0.055, 0, -0.02]), (4, 1)),
                    ['white', 'red', 'white', 'red']),
    *duplicate_body(frame_arm_3, 4, ROTATE_FOUR_EQUAL_SPACE,
                    np.tile(np.array([0.23, 0, -0.039]), (4, 1)),
                    ['white', 'red', 'white', 'red']),
    battery,
    *duplicate_body(esc, 4, ROTATE_FOUR_EQUAL_SPACE),
    *duplicate_body(motor, 4, ROTATE_FOUR_EQUAL_SPACE),
    *duplicate_body(propeller, 4, ROTATE_FOUR_EQUAL_SPACE),
    mounting_surface,
    *duplicate_body(mounting_spacers, 4, ROTATE_FOUR_EQUAL_SPACE),
    i2c_breadboard,
    raspberry_pi,
]

drone = MultiBody(
    name='Ugglan',
    bodies=_bodies
)
