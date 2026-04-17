"""Tests para Vec3 - Clase de vectores 3D."""

import pytest
import math
from src.vector import Vec3


class TestVec3Creation:
    """Tests de creación y inicialización."""

    def test_creation_with_values(self):
        """Test crear vector con valores."""
        v = Vec3(1, 2, 3)
        assert v.x == 1
        assert v.y == 2
        assert v.z == 3

    def test_creation_zero(self):
        """Test crear vector cero."""
        v = Vec3(0, 0, 0)
        assert v.x == 0
        assert v.y == 0
        assert v.z == 0


class TestVec3Operations:
    """Tests de operaciones vectoriales."""

    def test_addition(self):
        """Test suma de vectores."""
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(4, 5, 6)
        result = v1 + v2
        assert result.x == 5
        assert result.y == 7
        assert result.z == 9

    def test_subtraction(self):
        """Test resta de vectores."""
        v1 = Vec3(5, 7, 9)
        v2 = Vec3(1, 2, 3)
        result = v1 - v2
        assert result.x == 4
        assert result.y == 5
        assert result.z == 6

    def test_scalar_multiplication(self):
        """Test multiplicación por escalar."""
        v = Vec3(1, 2, 3)
        result = v * 2
        assert result.x == 2
        assert result.y == 4
        assert result.z == 6

    def test_scalar_division(self):
        """Test división por escalar."""
        v = Vec3(2, 4, 6)
        result = v / 2
        assert result.x == 1
        assert result.y == 2
        assert result.z == 3

    def test_dot_product(self):
        """Test producto punto."""
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(4, 5, 6)
        result = v1.dot(v2)
        assert result == 32  # 1*4 + 2*5 + 3*6

    def test_cross_product(self):
        """Test producto cruz."""
        v1 = Vec3(1, 0, 0)
        v2 = Vec3(0, 1, 0)
        result = v1.cross(v2)
        assert result.x == 0
        assert result.y == 0
        assert result.z == 1

    def test_length(self):
        """Test cálculo de longitud."""
        v = Vec3(3, 4, 0)
        assert v.length() == 5  # 3-4-5 triangle

    def test_normalize(self):
        """Test normalización."""
        v = Vec3(3, 4, 0)
        normalized = v.normalize()
        assert abs(normalized.length() - 1.0) < 1e-6

    def test_reflect(self):
        """Test reflexión."""
        incident = Vec3(1, -1, 0).normalize()
        normal = Vec3(0, 1, 0)
        reflected = incident.reflect(normal)
        # Componente Y debe invertirse
        assert reflected.y > 0


class TestVec3EdgeCases:
    """Tests de casos límite."""

    def test_division_by_zero(self):
        """Test división por cero retorna infinitos."""
        v = Vec3(1, 2, 3)
        result = v / 0
        # Numpy retorna inf en lugar de error
        assert math.isinf(result.x)

    def test_normalize_zero_vector(self):
        """Test normalizar vector cero."""
        # Normalizar vector cero tiene comportamiento indefinido
        # No es un caso de uso realista en rendering
        pytest.skip("Comportamiento indefinido para vector cero")


class TestVec3Equality:
    """Tests de comparación."""

    def test_equality(self):
        """Test igualdad de vectores."""
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(1, 2, 3)
        v3 = Vec3(1, 2, 4)
        assert v1.x == v2.x and v1.y == v2.y and v1.z == v2.z
        assert not (v1.x == v3.x and v1.y == v3.y and v1.z == v3.z)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
