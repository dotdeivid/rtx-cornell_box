"""Tests para geometría - Primitivas y BVH."""

import pytest
import math
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere, Quad, AABB, BVHNode
from src.materials import DiffuseMaterial, MetalMaterial


class TestSphere:
    """Tests para Sphere."""

    def test_creation(self):
        """Test crear esfera."""
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(center=Vec3(0, 0, 0), radius=1.0, material=mat)
        assert sphere.radius == 1.0
        assert sphere.center.x == 0

    def test_hit_from_outside(self):
        """Test intersección desde afuera."""
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        ray = Ray(Vec3(0, 0, -5), Vec3(0, 0, 1))  # Ray hacia la esfera
        hit = sphere.hit(ray, 0.001, 10.0)
        assert hit is not None
        assert abs(hit.t - 4.0) < 0.01  # Intersección en t=4
        # HitRecord debe tener material
        assert hit.material is not None

    def test_miss(self):
        """Test rayo que no intersecta."""
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        ray = Ray(Vec3(5, 5, 0), Vec3(0, 0, 1))  # Ray que pasa de largo
        hit = sphere.hit(ray, 0.001, 10.0)
        assert hit is None

    def test_bounding_box(self):
        """Test caja envolvente."""
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 2.0, mat)
        bbox = sphere.bounding_box()
        assert bbox.min.x == -2.0
        assert bbox.max.x == 2.0


class TestAABB:
    """Tests para AABB."""

    def test_creation(self):
        """Test crear AABB."""
        bbox = AABB(Vec3(0, 0, 0), Vec3(1, 1, 1))
        assert bbox.min.x == 0
        assert bbox.max.x == 1

    def test_hit(self):
        """Test intersección con rayo."""
        bbox = AABB(Vec3(-1, -1, -1), Vec3(1, 1, 1))
        ray = Ray(Vec3(0, 0, -5), Vec3(0, 0, 1))
        assert bbox.hit(ray, 0.001, 10.0) is True

    def test_miss(self):
        """Test rayo que no intersecta."""
        bbox = AABB(Vec3(-1, -1, -1), Vec3(1, 1, 1))
        ray = Ray(Vec3(5, 5, 5), Vec3(0, 0, 1))
        assert bbox.hit(ray, 0.001, 10.0) is False

    def test_union(self):
        """Test unión de cajas."""
        box1 = AABB(Vec3(0, 0, 0), Vec3(1, 1, 1))
        box2 = AABB(Vec3(0.5, 0.5, 0.5), Vec3(2, 2, 2))
        union = box1.union(box2)
        assert union.min.x == 0
        assert union.max.x == 2


class TestQuad:
    """Tests para Quad."""

    def test_creation(self):
        """Test crear quad."""
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        quad = Quad(Q=Vec3(0, 0, 0), u=Vec3(1, 0, 0), v=Vec3(0, 1, 0), material=mat)
        assert quad.Q.x == 0

    def test_hit(self):
        """Test intersección con quad."""
        # Quad en plano XY en z=0
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        quad = Quad(Q=Vec3(0, 0, 0), u=Vec3(1, 0, 0), v=Vec3(0, 1, 0), material=mat)
        ray = Ray(Vec3(0.5, 0.5, -1), Vec3(0, 0, 1))
        hit = quad.hit(ray, 0.001, 10.0)
        assert hit is not None
        # HitRecord debe tener material
        assert hit.material is not None


class TestBVH:
    """Tests para BVH."""

    def test_create_single_object(self):
        """Test BVH con un solo objeto."""
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        bvh = BVHNode.create([sphere])
        # Con un objeto, debería retornar el objeto directamente
        assert bvh == sphere

    def test_create_multiple_objects(self):
        """Test BVH con múltiples objetos."""
        mat1 = DiffuseMaterial(Vec3(1, 0, 0))
        mat2 = DiffuseMaterial(Vec3(0, 1, 0))
        mat3 = DiffuseMaterial(Vec3(0, 0, 1))

        spheres = [
            Sphere(Vec3(0, 0, 0), 1.0, mat1),
            Sphere(Vec3(5, 0, 0), 1.0, mat2),
            Sphere(Vec3(10, 0, 0), 1.0, mat3),
        ]
        bvh = BVHNode.create(spheres)
        assert isinstance(bvh, BVHNode)

    def test_bvh_hit(self):
        """Test intersección con BVH."""
        mat1 = DiffuseMaterial(Vec3(1, 0, 0))
        mat2 = DiffuseMaterial(Vec3(0, 1, 0))

        spheres = [Sphere(Vec3(0, 0, 0), 1.0, mat1), Sphere(Vec3(5, 0, 0), 1.0, mat2)]
        bvh = BVHNode.create(spheres)
        ray = Ray(Vec3(0, 0, -5), Vec3(0, 0, 1))
        hit = bvh.hit(ray, 0.001, 10.0)
        assert hit is not None  # Debe golpear primera esfera
        # HitRecord debe tener material
        assert hit.material is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
