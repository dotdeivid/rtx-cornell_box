"""Tests para sistema de cámara."""

import pytest
from src.vector import Vec3
from src.camera import Camera
from src.config.camera_config import CameraConfig


class TestCamera:
    """Tests para Camera."""

    def test_creation_default_config(self):
        config = CameraConfig()
        camera = Camera(config, aspect_ratio=16 / 9)
        assert camera.config.fov == 40.0
        # Por defecto la cámara es pinhole (sin DOF)
        assert camera.lens_radius == 0.0

    def test_creation_custom_config(self):
        config = CameraConfig(origin=Vec3(0, 0, -10), lookat=Vec3(0, 0, 0), fov=90.0)
        camera = Camera(config, aspect_ratio=1.0)
        assert camera.config.fov == 90.0

    def test_invalid_aspect_ratio_raises(self):
        with pytest.raises(ValueError):
            Camera(CameraConfig(), aspect_ratio=0.0)

    def test_get_ray_center_pinhole(self):
        config = CameraConfig(
            origin=Vec3(0, 0, -10), lookat=Vec3(0, 0, 0), aperture=0.0
        )
        camera = Camera(config, aspect_ratio=1.0)
        ray = camera.get_ray(0.5, 0.5)
        assert ray.origin == Vec3(0, 0, -10)
        assert abs(ray.direction.z - 1.0) < 1e-6

    def test_get_ray_different_positions(self):
        config = CameraConfig()
        camera = Camera(config, aspect_ratio=1.0)

        ray1 = camera.get_ray(0.0, 0.0)
        ray2 = camera.get_ray(1.0, 1.0)
        assert ray1.direction != ray2.direction

    def test_zenith_camera_parallel_vup_does_not_degenerate(self):
        """Vista cenital donde (origin - lookat) es paralelo a vup debe construir base válida."""
        config = CameraConfig(
            origin=Vec3(278, 900, 278),
            lookat=Vec3(278, 278, 278),
            vup=Vec3(0, 1, 0),
            aperture=0.0,
        )
        camera = Camera(config, aspect_ratio=1.0)
        assert abs(camera.u.length() - 1.0) < 1e-6
        assert abs(camera.v.length() - 1.0) < 1e-6
        assert abs(camera.w.length() - 1.0) < 1e-6
        ray = camera.get_ray(0.5, 0.5)
        assert abs(ray.direction.y + 1.0) < 1e-6

    def test_focus_distance_auto_and_manual(self):
        config_auto = CameraConfig(
            origin=Vec3(0, 0, -10), lookat=Vec3(0, 0, 0), focus_distance=None
        )
        assert Camera(config_auto, aspect_ratio=1.0).focus_dist == 10.0

        config_manual = CameraConfig(focus_distance=5.0)
        assert Camera(config_manual, aspect_ratio=1.0).focus_dist == 5.0


class TestCameraConfig:
    """Tests para CameraConfig."""

    def test_validation_fov(self):
        with pytest.raises(ValueError):
            CameraConfig(fov=0)
        with pytest.raises(ValueError):
            CameraConfig(fov=180)

    def test_validation_aperture_and_focus(self):
        with pytest.raises(ValueError):
            CameraConfig(aperture=-1)
        with pytest.raises(ValueError):
            CameraConfig(focus_distance=0)

    def test_validation_origin_lookat_and_vup(self):
        with pytest.raises(ValueError):
            CameraConfig(origin=Vec3(1, 1, 1), lookat=Vec3(1, 1, 1))
        with pytest.raises(ValueError):
            CameraConfig(vup=Vec3(0, 0, 0))

    def test_default_vectors_are_independent_instances(self):
        c1 = CameraConfig()
        c2 = CameraConfig()
        assert c1.origin is not c2.origin
        assert c1.lookat is not c2.lookat
        assert c1.vup is not c2.vup


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
