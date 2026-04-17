from src.vector import Vec3


class Ray:
    """
    Rayo en el espacio 3D: P(t) = origin + t * direction.

    La dirección se normaliza automáticamente para que t represente distancia real.

    Atributos:
        origin (Vec3): Punto de inicio del rayo
        direction (Vec3): Vector dirección (siempre unitario)
    """

    def __init__(self, origin: Vec3, direction: Vec3):
        """
        Args:
            origin: Punto de inicio del rayo
            direction: Dirección (se normaliza automáticamente)
        """
        self.origin = origin
        # Siempre normalizamos la dirección para evitar errores de escala
        self.direction = direction.normalize()

    def point_at(self, t: float) -> Vec3:
        """
        Punto en el rayo a distancia t: P(t) = origin + t * direction.

        Args:
            t: Distancia desde el origen

        Returns:
            Vec3: Punto en coordenadas 3D
        """
        return self.origin + self.direction * t
