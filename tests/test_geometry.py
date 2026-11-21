"""Tests para geometría - Primitivas, AABB y BVH."""

import pytest
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Geometry, Sphere, Quad, Triangle, AABB, BVHNode
from src.materials import DiffuseMaterial


class TestSphere:
    """Tests para Sphere."""

    def test_creation_and_inheritance(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(center=Vec3(0, 0, 0), radius=1.0, material=mat)
        assert isinstance(sphere, Geometry)
        assert sphere.radius == 1.0
        assert sphere.center == Vec3(0, 0, 0)

    def test_invalid_radius_raises(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        with pytest.raises(ValueError):
            Sphere(Vec3(0, 0, 0), 0.0, mat)
        with pytest.raises(ValueError):
            Sphere(Vec3(0, 0, 0), -2.0, mat)

    def test_hit_from_outside_and_inside(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)

        ray_out = Ray(Vec3(0, 0, -5), Vec3(0, 0, 1))
        hit_out = sphere.hit(ray_out, 0.001, 10.0)
        assert hit_out is not None
        assert abs(hit_out.t - 4.0) < 1e-6
        assert hit_out.front_face is True
        assert hit_out.normal == Vec3(0, 0, -1)
        assert hit_out.material is mat

        ray_in = Ray(Vec3(0, 0, 0), Vec3(0, 0, 1))
        hit_in = sphere.hit(ray_in, 0.001, 10.0)
        assert hit_in is not None
        assert abs(hit_in.t - 1.0) < 1e-6
        assert hit_in.front_face is False
        assert hit_in.normal == Vec3(0, 0, -1)

    def test_miss(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        ray = Ray(Vec3(5, 5, 0), Vec3(0, 0, 1))
        assert sphere.hit(ray, 0.001, 10.0) is None

    def test_bounding_box(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 2.0, mat)
        bbox = sphere.bounding_box()
        assert bbox.min == Vec3(-2, -2, -2)
        assert bbox.max == Vec3(2, 2, 2)

    def test_sample_solid_angle_outside_and_inside(self):
        import math

        mat = DiffuseMaterial(Vec3(1, 1, 1))
        sphere = Sphere(Vec3(0, 0, 0), 2.0, mat)

        direction, omega_out = sphere.sample_solid_angle(Vec3(0, 0, -10))
        assert abs(direction.length() - 1.0) < 1e-6
        assert 0.0 < omega_out < 2.0 * math.pi

        direction_in, omega_in = sphere.sample_solid_angle(Vec3(0, 0, 0))
        assert abs(direction_in.length() - 1.0) < 1e-6
        assert omega_in == pytest.approx(4.0 * math.pi)


class TestAABB:
    """Tests para AABB."""

    def test_creation_and_helpers(self):
        bbox = AABB(Vec3(0, 0, 0), Vec3(1, 4, 2))
        assert bbox.min == Vec3(0, 0, 0)
        assert bbox.max == Vec3(1, 4, 2)
        assert bbox.centroid() == Vec3(0.5, 2.0, 1.0)
        assert bbox.longest_axis() == 1

    def test_hit_and_miss(self):
        bbox = AABB(Vec3(-1, -1, -1), Vec3(1, 1, 1))
        assert bbox.hit(Ray(Vec3(0, 0, -5), Vec3(0, 0, 1)), 0.001, 10.0) is True
        assert bbox.hit(Ray(Vec3(5, 5, 5), Vec3(0, 0, 1)), 0.001, 10.0) is False

    def test_axis_aligned_ray_on_boundary_no_nan(self):
        """Rayos paralelos a ejes, incluso sobre el borde exacto, no deben fallar ni dar NaN."""
        bbox = AABB(Vec3(0, 0, 0), Vec3(2, 2, 2))
        # Rayo en plano x=0 (borde exacto), paralelo al eje Z
        ray_on_edge = Ray(Vec3(0, 1, -5), Vec3(0, 0, 1))
        assert bbox.hit(ray_on_edge, 0.001, 20.0) is True

        # Rayo fuera de la caja en X, paralelo al eje Z
        ray_outside = Ray(Vec3(-0.5, 1, -5), Vec3(0, 0, 1))
        assert bbox.hit(ray_outside, 0.001, 20.0) is False

    def test_union(self):
        box1 = AABB(Vec3(0, 0, 0), Vec3(1, 1, 1))
        box2 = AABB(Vec3(0.5, 0.5, 0.5), Vec3(2, 2, 2))
        union = box1.union(box2)
        assert union.min == Vec3(0, 0, 0)
        assert union.max == Vec3(2, 2, 2)


class TestQuad:
    """Tests para Quad."""

    def test_creation_and_inheritance(self):
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        quad = Quad(Q=Vec3(0, 0, 0), u=Vec3(2, 0, 0), v=Vec3(0, 3, 0), material=mat)
        assert isinstance(quad, Geometry)
        assert abs(quad.area - 6.0) < 1e-6
        assert quad.center == Vec3(1, 1.5, 0)

    def test_degenerate_quad_raises(self):
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        with pytest.raises(ValueError):
            Quad(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0), mat)

    def test_hit_front_and_back_face(self):
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        # Normal geométrica +z
        quad = Quad(Q=Vec3(0, 0, 0), u=Vec3(1, 0, 0), v=Vec3(0, 1, 0), material=mat)

        # Desde +z mirando hacia -z -> front_face True, normal +z
        hit_front = quad.hit(Ray(Vec3(0.5, 0.5, 1), Vec3(0, 0, -1)), 0.001, 10.0)
        assert hit_front is not None
        assert hit_front.front_face is True
        assert hit_front.normal == Vec3(0, 0, 1)

        # Desde -z mirando hacia +z -> front_face False, normal orientada hacia -z
        hit_back = quad.hit(Ray(Vec3(0.5, 0.5, -1), Vec3(0, 0, 1)), 0.001, 10.0)
        assert hit_back is not None
        assert hit_back.front_face is False
        assert hit_back.normal == Vec3(0, 0, -1)

    def test_sample_solid_angle_front_and_back(self):
        mat = DiffuseMaterial(Vec3(1, 1, 1))
        # Luz en y=10 con normal -y (emite hacia abajo)
        light = Quad(Vec3(0, 10, 0), Vec3(0, 0, 2), Vec3(2, 0, 0), mat)
        assert light.normal == Vec3(0, 1, 0)

        # Punto encima (+y, lado de la normal): recibe ángulo sólido positivo
        direction, solid_angle = light.sample_solid_angle(Vec3(1, 20, 1))
        assert abs(direction.length() - 1.0) < 1e-6
        assert solid_angle > 0.0

        # Punto debajo (-y, detrás de la cara emisora): ángulo sólido 0
        _, back_solid_angle = light.sample_solid_angle(Vec3(1, 0, 1))
        assert back_solid_angle == 0.0


class TestTriangle:
    """Tests para Triangle."""

    def test_hit_and_miss(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        tri = Triangle(Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(0, 2, 0), mat)
        assert isinstance(tri, Geometry)

        hit = tri.hit(Ray(Vec3(0.5, 0.5, 2), Vec3(0, 0, -1)), 0.001, 10.0)
        assert hit is not None
        assert abs(hit.t - 2.0) < 1e-6
        assert hit.front_face is True
        assert hit.normal == Vec3(0, 0, 1)

        # Desde la cara trasera: normal orientada contra el rayo
        hit_back = tri.hit(Ray(Vec3(0.5, 0.5, -2), Vec3(0, 0, 1)), 0.001, 10.0)
        assert hit_back is not None
        assert hit_back.front_face is False
        assert hit_back.normal == Vec3(0, 0, -1)

        # Fuera del triángulo
        assert tri.hit(Ray(Vec3(1.5, 1.5, 2), Vec3(0, 0, -1)), 0.001, 10.0) is None

    def test_bounding_box(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        tri = Triangle(Vec3(0, 0, 0), Vec3(1, 2, 3), Vec3(-1, 0, 1), mat)
        bbox = tri.bounding_box()
        assert bbox.min.x < -1.0
        assert bbox.max.z > 3.0


class TestBVH:
    """Tests para BVH."""

    def test_create_empty_raises(self):
        with pytest.raises(ValueError):
            BVHNode.create([])

    def test_create_single_object(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        bvh = BVHNode.create([sphere])
        assert bvh is sphere

    def test_create_does_not_mutate_input(self):
        mat = DiffuseMaterial(Vec3(1, 0, 0))
        s1 = Sphere(Vec3(10, 0, 0), 1.0, mat)
        s2 = Sphere(Vec3(0, 0, 0), 1.0, mat)
        s3 = Sphere(Vec3(5, 0, 0), 1.0, mat)
        spheres = [s1, s2, s3]
        bvh = BVHNode.create(spheres)
        assert isinstance(bvh, BVHNode)
        assert isinstance(bvh, Geometry)
        assert spheres == [s1, s2, s3]

    def test_bvh_hit_closest(self):
        mat1 = DiffuseMaterial(Vec3(1, 0, 0))
        mat2 = DiffuseMaterial(Vec3(0, 1, 0))
        s_near = Sphere(Vec3(0, 0, 0), 1.0, mat1)
        s_far = Sphere(Vec3(0, 0, 5), 1.0, mat2)

        bvh = BVHNode.create([s_far, s_near])
        ray = Ray(Vec3(0, 0, -5), Vec3(0, 0, 1))
        hit = bvh.hit(ray, 0.001, 20.0)
        assert hit is not None
        assert hit.obj_ref is s_near
        assert hit.material is mat1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
