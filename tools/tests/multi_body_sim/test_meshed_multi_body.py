import pyqtgraph.opengl as gl
import pytest

from ugglan_tools.multi_body_sim.meshed_multi_body import MeshedMultiBody
from unit_bodies import UNIT_CUBOID, UNIT_SPHERE, UNIT_CYLINDER


@pytest.mark.parametrize("unit_multi_body", [UNIT_CYLINDER, UNIT_CUBOID, UNIT_SPHERE])
def test_meshed_multi_body(unit_multi_body):
    meshed_multi_body = MeshedMultiBody(multi_body=unit_multi_body)

    for mesh_item in meshed_multi_body.iter_mesh_items():
        assert isinstance(mesh_item, gl.GLMeshItem)
