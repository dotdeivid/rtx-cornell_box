import numpy as np


class Vec3:
    """
    Vector tridimensional. Se usa para posiciones, direcciones y colores en la escena.

    Internamente usa numpy float64 para precisión numérica.
    """

    def __init__(self, x, y, z):
        """
        Args:
            x, y, z (float): Componentes del vector
        """
        # Usamos float64 para mayor precisión numérica
        self.components = np.array([x, y, z], dtype=np.float64)

    @property
    def x(self):
        """float: Componente X."""
        return self.components[0]

    @property
    def y(self):
        """float: Componente Y."""
        return self.components[1]

    @property
    def z(self):
        """float: Componente Z."""
        return self.components[2]

    # --- Operaciones Aritméticas Básicas ---

    def __add__(self, other):
        """Suma componente a componente. Retorna Vec3."""
        return Vec3(*(self.components + other.components))

    def __sub__(self, other):
        """Resta componente a componente. Retorna Vec3."""
        return Vec3(*(self.components - other.components))

    def __mul__(self, other):
        """Multiplicación por escalar o por otro Vec3 (componente a componente)."""
        # Multiplicación por escalar o por otro vector (elemento a elemento)
        if isinstance(other, Vec3):
            return Vec3(*(self.components * other.components))
        return Vec3(*(self.components * other))

    def __truediv__(self, scalar):
        """División por escalar componente a componente. Retorna Vec3."""
        return Vec3(*(self.components / scalar))

    # --- Operaciones de Álgebra Lineal Clave ---

    def dot(self, other):
        """
        Producto punto: v1·v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z

        Returns:
            float: Escalar resultante
        """
        return np.dot(self.components, other.components)

    def cross(self, other):
        """
        Producto cruz. El resultado es perpendicular a ambos operandos.

        Returns:
            Vec3: Vector perpendicular a self y other
        """
        return Vec3(*np.cross(self.components, other.components))

    def length(self):
        """
        Magnitud euclidiana: sqrt(x² + y² + z²)

        Returns:
            float: Longitud del vector (siempre ≥ 0)
        """
        return np.linalg.norm(self.components)

    def normalize(self):
        """
        Vector unitario con la misma dirección (magnitud = 1).

        Returns:
            Vec3: Vector normalizado, o Vec3(0,0,0) si la magnitud es 0
        """
        mag = self.length()
        if mag == 0:
            return Vec3(0, 0, 0)
        return self / mag

    def reflect(self, normal):
        """
        Reflexión especular: R = I - 2(I·N)N

        Args:
            normal (Vec3): Normal de la superficie (debe ser unitaria)

        Returns:
            Vec3: Vector reflejado
        """
        return self - normal * (2 * self.dot(normal))

    def __repr__(self):
        return f"Vec3({self.x}, {self.y}, {self.z})"
