"""Tests para sistema de cámara."""

import pytest
from src.vector import Vec3
from src.camera import Camera
from src.config.camera_config import CameraConfig


class TestCamera:
    """Tests para Camera."""

    def test_creation_default_config(self):
        """Test crear cámara con config default."""
        config = CameraConfig()
        camera = Camera(config, aspect_ratio=16 / 9)
        assert camera.config.fov == 40.0

    def test_creation_custom_config(self):
        """Test crear cámara con config custom."""
        config = CameraConfig(origin=Vec3(0, 0, -10), lookat=Vec3(0, 0, 0), fov=90.0)
        camera = Camera(config, aspect_ratio=1.0)
        assert camera.config.fov == 90.0

    def test_get_ray_center(self):
        """Test generar rayo al centro."""
        config = CameraConfig()
        camera = Camera(config, aspect_ratio=1.0)
        ray = camera.get_ray(0.5, 0.5)  # Centro
        assert ray is not None
        assert ray.origin is not None
        assert ray.direction is not None

    def test_get_ray_different_positions(self):
        """Test generar rayos en diferentes posiciones."""
        config = CameraConfig()
        camera = Camera(config, aspect_ratio=1.0)

        ray1 = camera.get_ray(0.0, 0.0)  # Esquina inferior izquierda
        ray2 = camera.get_ray(1.0, 1.0)  # Esquina superior derecha
        ray3 = camera.get_ray(0.5, 0.5)  # Centro

        # Las direcciones deben ser diferentes
        assert (
            ray1.direction.x != ray2.direction.x or ray1.direction.y != ray2.direction.y
        )

    def test_focus_distance_auto(self):
        """Test distancia focal automática."""
        config = CameraConfig(
            origin=Vec3(0, 0, -10), lookat=Vec3(0, 0, 0), focus_distance=None  # Auto
        )
        camera = Camera(config, aspect_ratio=1.0)
        assert camera.focus_dist == 10.0  # Distancia calculada

    def test_focus_distance_manual(self):
        """Test distancia focal manual."""
        config = CameraConfig(focus_distance=5.0)
        camera = Camera(config, aspect_ratio=1.0)
        assert camera.focus_dist == 5.0


class TestCameraConfig:
    """Tests para CameraConfig."""

    def test_validation_fov(self):
        """Test validación de FOV."""
        with pytest.raises(ValueError):
            CameraConfig(fov=0)  # FOV debe estar entre 0 y 180

        with pytest.raises(ValueError):
            CameraConfig(fov=180)  # Exactamente 180 también inválido

    def test_validation_aperture(self):
        """Test validación de aperture."""
        with pytest.raises(ValueError):
            CameraConfig(aperture=-1)  # Aperture no puede ser negativo

    def test_valid_config(self):
        """Test config válida."""
        config = CameraConfig(fov=45.0, aperture=10.0)
        assert config.fov == 45.0
        assert config.aperture == 10.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
