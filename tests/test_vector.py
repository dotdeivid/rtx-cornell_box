"""Tests para Vec3 - Clase de vectores 3D."""

import pytest
from src.vector import Vec3


class TestVec3Creation:
    """Tests de creación e inicialización."""

    def test_creation_with_values(self):
        v = Vec3(1, 2, 3)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_creation_zero(self):
        v = Vec3(0, 0, 0)
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0


class TestVec3Operations:
    """Tests de operaciones vectoriales."""

    def test_addition(self):
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(4, 5, 6)
        assert v1 + v2 == Vec3(5, 7, 9)

    def test_subtraction(self):
        v1 = Vec3(5, 7, 9)
        v2 = Vec3(1, 2, 3)
        assert v1 - v2 == Vec3(4, 5, 6)

    def test_negation(self):
        v = Vec3(1, -2, 3)
        assert -v == Vec3(-1, 2, -3)

    def test_scalar_multiplication_right_and_left(self):
        v = Vec3(1, 2, 3)
        assert v * 2 == Vec3(2, 4, 6)
        assert 2 * v == Vec3(2, 4, 6)

    def test_hadamard_multiplication(self):
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(4, 5, 6)
        assert v1 * v2 == Vec3(4, 10, 18)

    def test_scalar_division(self):
        v = Vec3(2, 4, 6)
        assert v / 2 == Vec3(1, 2, 3)

    def test_dot_product(self):
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(4, 5, 6)
        assert v1.dot(v2) == 32.0

    def test_cross_product(self):
        v1 = Vec3(1, 0, 0)
        v2 = Vec3(0, 1, 0)
        assert v1.cross(v2) == Vec3(0, 0, 1)

    def test_length_and_length_squared(self):
        v = Vec3(3, 4, 0)
        assert v.length_squared() == 25.0
        assert v.length() == 5.0

    def test_normalize(self):
        v = Vec3(3, 4, 0)
        normalized = v.normalize()
        assert abs(normalized.length() - 1.0) < 1e-6

    def test_reflect(self):
        incident = Vec3(1, -1, 0).normalize()
        normal = Vec3(0, 1, 0)
        reflected = incident.reflect(normal)
        assert reflected.y > 0
        assert abs(reflected.x - incident.x) < 1e-6


class TestVec3EdgeCases:
    """Tests de casos límite."""

    def test_division_by_zero_raises(self):
        v = Vec3(1, 2, 3)
        with pytest.raises(ZeroDivisionError):
            _ = v / 0

    def test_normalize_zero_vector_returns_zero(self):
        v = Vec3(0, 0, 0)
        assert v.normalize() == Vec3(0, 0, 0)


class TestVec3Equality:
    """Tests de comparación."""

    def test_equality(self):
        v1 = Vec3(1, 2, 3)
        v2 = Vec3(1, 2, 3)
        v3 = Vec3(1, 2, 4)
        assert v1 == v2
        assert v1 != v3
        assert v1 != (1, 2, 3)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
