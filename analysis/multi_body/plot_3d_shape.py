from enum import Enum
from typing import Union, List

import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import euclidean_transform

DEFAULT_ROTATION = np.array([0, 0, 0])
DEFAULT_POSITION = np.array([0, 0, 0])

EDGE_COLOR = 'black'
FACE_ALPHA = 0.5
LINE_WIDTH = 0.5

CYLINDER_N_SIDES = 20

AXES_MARGIN = 1.1


class TransformType(Enum):
    TRANSLATE = 0
    SCALE = 1
    ROTATE = 2


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
class Face:
    x: np.array
    y: np.array
    z: np.array

    def to_vertices(self):
        vertices = [list(zip(self.x, self.y, self.z))]
        return vertices

    def transform(self, transform_type: TransformType, *args, **kwargs):
        v_org = np.array([self.x, self.y, self.z])

        if transform_type == TransformType.TRANSLATE:
            v_rot = euclidean_transform.translate(v_org, *args, **kwargs)

        if transform_type == TransformType.SCALE:
            v_rot = euclidean_transform.scale(v_org, *args, **kwargs)

        if transform_type == TransformType.ROTATE:
            v_rot = euclidean_transform.rotate(v_org, *args, **kwargs)

        self.x, self.y, self.z = v_rot


def plot_3d_shape(axes: plt.axes,
                  shape: Union[Cuboid, Cylinder],
                  rotation: np.array = DEFAULT_ROTATION,
                  translation: np.array = DEFAULT_POSITION,
                  color: Union[str, List[float]] = None):
    """
    Plots a 3d shape on the provided axis.

    Note, the rotation is performed after the
    translation.

    Also, due to issues with mplot3d some objects
    might not be shown correctly at certain angles,
    see https://matplotlib.org/mpl_toolkits/mplot3d/faq.html
    """
    if isinstance(shape, Cuboid):
        faces = _cuboid_faces(shape)
    elif isinstance(shape, Cylinder):
        faces = _cylinder_faces(shape)

    _transform_faces_to_axes(axes, faces, color, rotation, translation)


def _transform_faces_to_axes(axes, faces, color, rotation, translation):
    for face in faces:
        face.transform(TransformType.TRANSLATE, *translation)
        face.transform(TransformType.ROTATE, *rotation)

        axes.add_collection3d(Poly3DCollection(face.to_vertices(),
                                               alpha=FACE_ALPHA,
                                               edgecolor=EDGE_COLOR,
                                               linewidth=LINE_WIDTH,
                                               facecolor=color))


def _cuboid_faces(shape: Cuboid):
    xy_unit_face_x = np.array([-0.5, 0.5, 0.5, -0.5])
    xy_unit_face_y = np.array([-0.5, -0.5, 0.5, 0.5])
    xy_unit_face_z = np.array([-0.5, -0.5, -0.5, -0.5])

    scale = {'x_s': shape.length_x, 'y_s': shape.width_y, 'z_s': shape.height_z}

    transformations = [
        ({'z_t': 0}, {'z_rad': 0}),
        ({'y_t': 0}, {'x_rad': -np.pi / 2}),
        ({'x_t': 0}, {'y_rad': np.pi / 2}),
        ({'z_t': 1}, {'z_rad': 0}),
        ({'y_t': 1}, {'x_rad': -np.pi / 2}),
        ({'x_t': 1}, {'y_rad': np.pi / 2}),
    ]

    def iter_faces():
        for t, r in transformations:
            face = Face(xy_unit_face_x, xy_unit_face_y, xy_unit_face_z)

            face.transform(TransformType.ROTATE, **r)
            face.transform(TransformType.TRANSLATE, **t)
            face.transform(TransformType.SCALE, **scale)

            yield face

    return list(iter_faces())


def _cylinder_faces(shape: Cylinder):
    s = np.linspace(0, 2 * np.pi, CYLINDER_N_SIDES)
    x = np.cos(s) * shape.radius_xy
    y = np.sin(s) * shape.radius_xy

    def face_top():
        z_top = (np.zeros(CYLINDER_N_SIDES) + 0.5) * shape.height_z
        return Face(x, y, z_top)

    def face_bottom():
        z_bottom = (np.zeros(CYLINDER_N_SIDES) - 0.5) * shape.height_z
        return Face(x, y, z_bottom)

    def iter_face_sides():
        for i in range(CYLINDER_N_SIDES - 1):
            x_side = np.array([x[i], x[i + 1], x[i + 1], x[i]])
            y_side = np.array([y[i], y[i + 1], y[i + 1], y[i]])
            z_side = np.array([-0.5, -0.5, 0.5, 0.5]) * shape.height_z

            yield Face(x_side, y_side, z_side)

    def iter_faces():
        yield face_top()
        yield face_bottom()
        yield from iter_face_sides()

    return list(iter_faces())
