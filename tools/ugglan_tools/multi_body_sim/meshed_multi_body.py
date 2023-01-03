from dataclasses import dataclass
from typing import List, Union

import numpy as np
import pyqtgraph.opengl as gl
from PyQt6.QtGui import QColor
from PyQt6.QtGui import QMatrix4x4

import ugglan_tools.multi_body_sim.euclidean_transform as euc_trans
from ugglan_tools.multi_body_sim.multi_body import MultiBody
from ugglan_tools.multi_body_sim.shapes import Cuboid, Cylinder, Sphere, Shapes

EDGE_COLOR = QColor("black")

FACE_ALPHA = int(255 * 0.8)

SPHERE_RES = 15
CYLINDER_RES = 20


@dataclass
class FaceVert:
    vertices: np.ndarray = None
    faces: np.ndarray = None

    _is_init: bool = False

    def __post_init__(self):
        if self.vertices is not None and self.faces is not None:
            self._is_init = True

    def __add__(self, other):
        if self._is_init and other._is_init:
            vertices = np.vstack((self.vertices, other.vertices))

            other_face_start_idx = np.max(self.faces) + 1
            faces = np.vstack((self.faces, other.faces + other_face_start_idx))

            return FaceVert(vertices, faces)
        elif self._is_init:
            return self
        else:
            return other


class MeshedMultiBody:
    def __init__(self, multi_body: MultiBody):
        self._multi_body = multi_body
        self._mesh_items = []

        self._create_mesh_items()

    def iter_mesh_items(self):
        return iter(self._mesh_items)

    def set_transform(self, transform: QMatrix4x4):
        for item in self._mesh_items:
            item.setTransform(transform)

    def _create_mesh_items(self):
        for body in self._multi_body.bodies:
            self._mesh_items.append(self._mesh_body(shape=body.shape_m,
                                                    rot_b_rad=body.rot_b_frame_rad,
                                                    trans_i_m=body.trans_i_frame_m,
                                                    rot_i_rad=body.rot_i_frame_rad,
                                                    color=body.color))

    def _mesh_body(self, shape: Shapes,
                   rot_b_rad: np.array = np.array([0, 0, 0]),
                   trans_i_m: np.array = np.array([0, 0, 0]),
                   rot_i_rad: np.array = np.array([0, 0, 0]),
                   color: Union[QColor, List[int]] = QColor('black')):

        if isinstance(shape, Cuboid):
            mesh_data = self._mesh_data_cuboid(shape)

        elif isinstance(shape, Sphere):
            mesh_data = self._mesh_data_sphere(shape)

        elif isinstance(shape, Cylinder):
            mesh_data = self._mesh_data_cylinder(shape)

        else:
            raise ValueError("Invalid shape")

        self._mesh_data_transform(mesh_data, rot_b_rad, trans_i_m, rot_i_rad)
        mesh_item = gl.GLMeshItem(meshdata=mesh_data, color=self._parse_color(color),
                                  smooth=True, computeNormals=False, glOptions='translucent')

        return mesh_item

    @staticmethod
    def _mesh_data_cuboid(cuboid: Cuboid) -> gl.MeshData:
        unit_vertices = np.array([[0.5, 0.5, 0.0],
                                  [0.5, -0.5, 0.0],
                                  [-0.5, -0.5, 0.0],
                                  [-0.5, 0.5, 0.0]])
        unit_faces = np.array([[0, 1, 2],
                               [0, 2, 3]])

        transformations = [
            ({'x_s': cuboid.length_x, 'y_s': cuboid.width_y}, {'z_t': 0.5 * cuboid.height_z}, {}),
            ({'x_s': cuboid.length_x, 'y_s': cuboid.width_y}, {'z_t': -0.5 * cuboid.height_z}, {}),

            ({'x_s': cuboid.length_x, 'y_s': cuboid.height_z}, {'z_t': 0.5 * cuboid.width_y}, {'x_rad': np.pi / 2}),
            ({'x_s': cuboid.length_x, 'y_s': cuboid.height_z}, {'z_t': 0.5 * cuboid.width_y}, {'x_rad': -np.pi / 2}),

            ({'x_s': cuboid.height_z, 'y_s': cuboid.width_y}, {'z_t': 0.5 * cuboid.length_x}, {'y_rad': np.pi / 2}),
            ({'x_s': cuboid.height_z, 'y_s': cuboid.width_y}, {'z_t': 0.5 * cuboid.length_x}, {'y_rad': -np.pi / 2}),
        ]

        def get_cuboid():
            face_vert = FaceVert()

            for s, t, r in transformations:
                vertices_t = unit_vertices.transpose()

                vertices_t = euc_trans.scale(vertices_t, **s)
                vertices_t = euc_trans.translate(vertices_t, **t)
                vertices_t = euc_trans.rotate(vertices_t, **r)

                face_vert += FaceVert(vertices_t.transpose(), unit_faces)

            return face_vert

        cuboid_fc = get_cuboid()

        return gl.MeshData(vertexes=cuboid_fc.vertices, faces=cuboid_fc.faces)

    @staticmethod
    def _mesh_data_sphere(sphere: Sphere) -> gl.MeshData:
        return gl.MeshData.sphere(rows=SPHERE_RES, cols=SPHERE_RES, radius=sphere.radius)

    @staticmethod
    def _mesh_data_cylinder(cylinder: Cylinder) -> gl.MeshData:
        def get_side():
            md_sides = gl.MeshData.cylinder(rows=CYLINDER_RES, cols=CYLINDER_RES,
                                            radius=[cylinder.radius_xy, cylinder.radius_xy], length=cylinder.height_z)
            vertices = md_sides.vertexes()
            vertices[:, 2] -= 0.5 * cylinder.height_z

            return FaceVert(vertices=vertices, faces=md_sides.faces())

        def get_disc(z):
            s = np.linspace(0, 2 * np.pi, CYLINDER_RES)
            x = np.cos(s) * cylinder.radius_xy
            y = np.sin(s) * cylinder.radius_xy

            vertices = np.column_stack((x, y, z * np.ones(CYLINDER_RES)))
            vertices = np.vstack([vertices, [0, 0, z]])

            faces = np.column_stack((np.arange(0, CYLINDER_RES - 1),
                                     np.arange(1, CYLINDER_RES),
                                     CYLINDER_RES * np.ones(CYLINDER_RES - 1)))
            faces = faces.astype(int)

            return FaceVert(vertices=vertices, faces=faces)

        cylinder_fc = get_side() + get_disc(0.5 * cylinder.height_z) + get_disc(-0.5 * cylinder.height_z)

        return gl.MeshData(vertexes=cylinder_fc.vertices, faces=cylinder_fc.faces)

    @staticmethod
    def _mesh_data_transform(mesh_data: gl.MeshData,
                             rot_b_rad: np.array,
                             trans_i_m: np.array,
                             rot_i_rad: np.array):
        vertices_t = mesh_data.vertexes().transpose()

        vertices_t = euc_trans.rotate(vertices_t, *rot_b_rad)
        vertices_t = euc_trans.translate(vertices_t, *trans_i_m)
        vertices_t = euc_trans.rotate(vertices_t, *rot_i_rad)

        mesh_data.setVertexes(vertices_t.transpose())

    @staticmethod
    def _parse_color(color: Union[QColor, List[int]]):
        if isinstance(color, list):
            return QColor(*color, alpha=FACE_ALPHA)

        elif isinstance(color, QColor):
            color.setAlpha(FACE_ALPHA)
            return color

        else:
            raise ValueError
