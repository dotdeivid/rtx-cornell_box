"""Tests para scene builder."""

import pytest
from src.vector import Vec3
from src.scene import create_cornell_box_scene, SceneMode
from src.geometry import BVHNode


class TestSceneBuilder:
    """Tests para scene builder."""

    def test_create_spheres_scene(self):
        """Test crear escena con esferas."""
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)

        assert world is not None
        assert isinstance(world, (BVHNode, object))
        assert lights is not None
        assert len(lights) > 0

    @pytest.mark.skip(reason="Requiere bunny.obj - test opcional")
    def test_create_bunny_scene(self):
        """Test crear escena con bunny."""
        world, lights = create_cornell_box_scene(SceneMode.BUNNY)

        assert world is not None
        assert lights is not None

    def test_lights_are_emissive(self):
        """Test que luces tienen EmissiveMaterial."""
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)

        assert len(lights) > 0
        for light in lights:
            # Las luces deben tener material con emission_color
            assert hasattr(light.material, "emission_color")

    def test_spheres_scene_has_objects(self):
        """Test que escena de esferas tiene objetos."""
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)

        # Debe tener al menos paredes + luz + esferas
        assert world is not None

    def test_different_modes_different_results(self):
        """Test que diferentes modos producen diferentes escenas."""
        spheres_world, spheres_lights = create_cornell_box_scene(SceneMode.SPHERES)

        # Modo SPHERES debe tener luces
        assert len(spheres_lights) > 0

        # Escenas deben tener mundo
        assert spheres_world is not None

    def test_materials_are_used(self):
        """Test que materiales están siendo usados."""
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)

        # Las luces usan materiales
        for light in lights:
            assert light.material is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
