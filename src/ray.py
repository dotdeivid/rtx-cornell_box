from src.vector import Vec3

class Ray:
    def __init__(self, origin: Vec3, direction: Vec3):
        self.origin = origin
        # Siempre normalizamos la dirección para evitar errores de escala
        self.direction = direction.normalize()

    def point_at(self, t: float) -> Vec3:
        """Calcula el punto P(t) en la recta del rayo."""
        return self.origin + self.direction * t