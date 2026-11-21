"""Tests para sistema de materiales."""

import pytest
from src.vector import Vec3
from src.ray import Ray
from src.geometry import HitRecord
from src.materials import (
    Material,
    DiffuseMaterial,
    MetalMaterial,
    DielectricMaterial,
    EmissiveMaterial,
)


class TestMaterialBase:
    """Tests para Material ABC."""

    def test_cannot_instantiate_abstract(self):
        with pytest.raises(TypeError):
            Material()  # type: ignore[abstract]


class TestDiffuseMaterial:
    """Tests para DiffuseMaterial."""

    def test_creation(self):
        mat = DiffuseMaterial(Vec3(0.8, 0.3, 0.3))
        assert mat.albedo == Vec3(0.8, 0.3, 0.3)

    def test_scatter(self):
        mat = DiffuseMaterial(Vec3(0.5, 0.5, 0.5))
        ray = Ray(Vec3(0, 0, -1), Vec3(0, 0, 1))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 0, -1))

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)
        assert scattered is True
        assert attenuation == Vec3(0.5, 0.5, 0.5)
        assert scattered_ray is not None
        assert scattered_ray.direction.dot(hit.normal) >= 0.0

    def test_no_emission(self):
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        assert mat.emitted() == Vec3(0, 0, 0)
        assert mat.uses_nee is True


class TestMetalMaterial:
    """Tests para MetalMaterial."""

    def test_creation_and_fuzz_clamping(self):
        assert MetalMaterial(Vec3(0.8, 0.8, 0.8), fuzz=0.3).fuzz == 0.3
        assert MetalMaterial(Vec3(1, 1, 1), fuzz=2.0).fuzz == 1.0
        assert MetalMaterial(Vec3(1, 1, 1), fuzz=-0.5).fuzz == 0.0

    def test_perfect_reflection(self):
        mat = MetalMaterial(Vec3(1, 1, 1), fuzz=0.0)
        ray = Ray(Vec3(0, 1, 0), Vec3(1, -1, 0))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 1, 0))

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)
        assert scattered is True
        assert attenuation == Vec3(1, 1, 1)
        assert scattered_ray is not None
        assert scattered_ray.direction.y > 0

    def test_absorbs_when_ray_below_surface(self):
        mat = MetalMaterial(Vec3(1, 1, 1), fuzz=0.0)
        # Rayo saliendo desde atrás contra normal en la misma dirección
        ray = Ray(Vec3(0, -1, 0), Vec3(0, 1, 0))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 1, 0))
        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)
        assert scattered is False
        assert attenuation is None
        assert scattered_ray is None

    def test_no_emission(self):
        mat = MetalMaterial(Vec3(1, 1, 1))
        assert mat.emitted().length() == 0.0
        assert mat.uses_nee is False


class TestDielectricMaterial:
    """Tests para DielectricMaterial."""

    def test_creation_and_validation(self):
        assert DielectricMaterial(ior=1.5).ior == 1.5
        assert DielectricMaterial(ior=1.33).ior == 1.33
        with pytest.raises(ValueError):
            DielectricMaterial(ior=0.0)
        with pytest.raises(ValueError):
            DielectricMaterial(ior=-1.5)

    def test_scatter_returns_white_attenuation(self):
        mat = DielectricMaterial(ior=1.5)
        ray = Ray(Vec3(0, 1, 0), Vec3(0, -1, 0))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 1, 0))

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)
        assert scattered is True
        assert attenuation == Vec3(1.0, 1.0, 1.0)
        assert scattered_ray is not None

    def test_total_internal_reflection(self):
        """Saliendo de vidrio (ior=1.5) a ángulo mayor al crítico (~41.8°) siempre refleja."""
        mat = DielectricMaterial(ior=1.5)
        # Rayo saliendo a ~60° respecto a la normal interior (0, -1, 0)
        ray = Ray(Vec3(0, 0, 0), Vec3(0.866, 0.5, 0.0))
        # front_face=False con normal ya orientada hacia adentro (0, -1, 0)
        hit = HitRecord(
            t=1.0,
            point=Vec3(0, 1, 0),
            normal=Vec3(0, -1, 0),
            front_face=False,
        )
        for _ in range(10):
            scattered, _, scattered_ray = mat.scatter(ray, hit)
            assert scattered is True
            assert scattered_ray is not None
            # Reflejado hacia adentro (y < 0)
            assert scattered_ray.direction.y < 0.0

    def test_reflectance_bounds(self):
        assert DielectricMaterial._reflectance(0.0, 1.5) > 0.9
        assert DielectricMaterial._reflectance(1.0, 1.5) < 0.1

    def test_no_emission(self):
        mat = DielectricMaterial(ior=1.5)
        assert mat.emitted().length() == 0.0
        assert mat.uses_nee is False


class TestEmissiveMaterial:
    """Tests para EmissiveMaterial."""

    def test_emits_and_does_not_scatter(self):
        mat = EmissiveMaterial(Vec3(15, 15, 15))
        assert mat.emitted() == Vec3(15, 15, 15)

        ray = Ray(Vec3(0, 0, -1), Vec3(0, 0, 1))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 0, -1))
        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)
        assert scattered is False
        assert attenuation is None
        assert scattered_ray is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
