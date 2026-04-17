"""Tests para sistema de materiales."""

import pytest
import math
from src.vector import Vec3
from src.ray import Ray
from src.materials import (
    Material,
    HitRecord,
    DiffuseMaterial,
    MetalMaterial,
    DielectricMaterial,
)


class TestMaterialBase:
    """Tests para Material ABC."""

    def test_cannot_instantiate_abstract(self):
        """No se puede instanciar Material directamente."""
        with pytest.raises(TypeError):
            Material()


class TestDiffuseMaterial:
    """Tests para DiffuseMaterial."""

    def test_creation(self):
        """Test crear material difuso."""
        albedo = Vec3(0.8, 0.3, 0.3)
        mat = DiffuseMaterial(albedo)
        assert mat.albedo.x == 0.8
        assert mat.albedo.y == 0.3
        assert mat.albedo.z == 0.3

    def test_scatter(self):
        """Test dispersión difusa."""
        mat = DiffuseMaterial(Vec3(0.5, 0.5, 0.5))

        ray = Ray(Vec3(0, 0, -1), Vec3(0, 0, 1))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 0, -1))

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)

        assert scattered is True
        assert attenuation is not None
        assert scattered_ray is not None
        assert attenuation.x == 0.5

    def test_no_emission(self):
        """Material difuso no emite luz."""
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        emission = mat.emitted()
        assert emission.x == 0
        assert emission.y == 0
        assert emission.z == 0


class TestMetalMaterial:
    """Tests para MetalMaterial."""

    def test_creation(self):
        """Test crear material metálico."""
        mat = MetalMaterial(Vec3(0.8, 0.8, 0.8), fuzz=0.3)
        assert mat.fuzz == 0.3

    def test_fuzz_clamped(self):
        """Fuzz se clampea a [0, 1]."""
        mat = MetalMaterial(Vec3(1, 1, 1), fuzz=2.0)
        assert mat.fuzz == 1.0

    def test_perfect_reflection(self):
        """Test reflexión perfecta (sin fuzz)."""
        mat = MetalMaterial(Vec3(1, 1, 1), fuzz=0.0)

        # Rayo incidente vertical hacia abajo
        ray = Ray(Vec3(0, 1, 0), Vec3(0, -1, 0))
        hit = HitRecord(
            t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 1, 0)  # Normal hacia arriba
        )

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)

        assert scattered is True
        # El rayo debe reflejarse hacia arriba
        assert scattered_ray.direction.y > 0

    def test_no_emission(self):
        """Material metálico no emite luz."""
        mat = MetalMaterial(Vec3(1, 1, 1))
        emission = mat.emitted()
        assert emission.length() == 0


class TestDielectricMaterial:
    """Tests para DielectricMaterial."""

    def test_creation(self):
        """Test crear material dieléctrico."""
        mat = DielectricMaterial(ior=1.5)
        assert mat.ior == 1.5

    def test_glass(self):
        """Test vidrio (ior=1.5)."""
        mat = DielectricMaterial(ior=1.5)
        assert mat.ior == 1.5

    def test_water(self):
        """Test agua (ior=1.33)."""
        mat = DielectricMaterial(ior=1.33)
        assert mat.ior == 1.33

    def test_scatter_always_returns_white(self):
        """Vidrio no atenúa color (retorna blanco)."""
        mat = DielectricMaterial(ior=1.5)

        ray = Ray(Vec3(0, 1, 0), Vec3(0, -1, 0))
        hit = HitRecord(t=1.0, point=Vec3(0, 0, 0), normal=Vec3(0, 1, 0))

        scattered, attenuation, scattered_ray = mat.scatter(ray, hit)

        assert scattered is True
        assert attenuation is not None
        # Vidrio no atenúa
        assert attenuation.x == 1.0
        assert attenuation.y == 1.0
        assert attenuation.z == 1.0

    def test_reflectance_at_grazing_angle(self):
        """Test reflectancia de Fresnel en ángulo rasante."""
        # A ángulo rasante (cosine ≈ 0), reflectancia debe ser alta
        reflectance = DielectricMaterial._reflectance(0.0, 1.5)
        assert reflectance > 0.9  # Alta probabilidad de reflexión

    def test_reflectance_at_normal_incidence(self):
        """Test reflectancia de Fresnel en incidencia normal."""
        # En incidencia normal (cosine = 1), reflectancia es mínima
        reflectance = DielectricMaterial._reflectance(1.0, 1.5)
        assert reflectance < 0.1  # Baja probabilidad de reflexión

    def test_no_emission(self):
        """Material dieléctrico no emite luz."""
        mat = DielectricMaterial(ior=1.5)
        emission = mat.emitted()
        assert emission.length() == 0


class TestMaterialsIntegration:
    """Tests de integración entre materiales."""

    def test_different_materials_different_behavior(self):
        """Cada material se comporta diferente."""
        diffuse = DiffuseMaterial(Vec3(0.5, 0.5, 0.5))
        metal = MetalMaterial(Vec3(0.8, 0.8, 0.8))
        glass = DielectricMaterial(ior=1.5)

        # Todos no emiten
        assert diffuse.emitted().length() == 0
        assert metal.emitted().length() == 0
        assert glass.emitted().length() == 0

        # Pero tienen diferentes propiedades
        assert hasattr(metal, "fuzz")
        assert hasattr(glass, "ior")
        assert hasattr(diffuse, "albedo")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
