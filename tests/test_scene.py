"""Tests para scene builder."""

import os

import pytest
from src.geometry import BVHNode
from src.scene.cornell_box import _get_bunny_objects
from src.materials import EmissiveMaterial
from src.scene import create_cornell_box_scene, SceneMode


class TestSceneBuilder:
    """Tests para scene builder."""

    def test_create_spheres_scene(self):
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)
        assert isinstance(world, BVHNode)
        assert len(lights) == 1
        assert isinstance(lights[0].material, EmissiveMaterial)

    def test_create_boxes_scene(self):
        world, lights = create_cornell_box_scene(SceneMode.BOXES)
        assert isinstance(world, BVHNode)
        assert len(lights) == 1
        assert isinstance(lights[0].material, EmissiveMaterial)

    def test_create_bunny_scene(self):
        world, lights = create_cornell_box_scene(SceneMode.BUNNY)
        assert isinstance(world, BVHNode)
        assert len(lights) == 1
        assert isinstance(lights[0].material, EmissiveMaterial)

    def test_bunny_rests_on_floor_from_any_cwd(self, tmp_path):
        cwd = os.getcwd()
        os.chdir(tmp_path)
        try:
            tris = _get_bunny_objects()
        finally:
            os.chdir(cwd)
        min_y = min(min(t.v0.y, t.v1.y, t.v2.y) for t in tris)
        assert len(tris) == 4968
        assert min_y == pytest.approx(0.0)

    def test_lights_emit_positive_radiance(self):
        _, lights = create_cornell_box_scene(SceneMode.SPHERES)
        for light in lights:
            assert light.material.emitted().length() > 0.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
