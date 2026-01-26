import numpy as np

# Clase que representa un vector de 3 dimensiones en la cual haremos operaciones para simular un vector en 3D
# Almacena 3 números en punto flotante que representan los ejes X, Y y Z 
class Vec3:
    def __init__(self, x, y, z):
        # Usamos float64 para mayor precisión numérica
        self.components = np.array([x, y, z], dtype=np.float64)

    @property
    def x(self): return self.components[0]
    @property
    def y(self): return self.components[1]
    @property
    def z(self): return self.components[2]

    # --- Operaciones Aritméticas Básicas ---
    def __add__(self, other):
        return Vec3(*(self.components + other.components))

    def __sub__(self, other):
        return Vec3(*(self.components - other.components))

    def __mul__(self, other):
        # Multiplicación por escalar o por otro vector (elemento a elemento)
        if isinstance(other, Vec3):
            return Vec3(*(self.components * other.components))
        return Vec3(*(self.components * other))

    def __truediv__(self, scalar):
        return Vec3(*(self.components / scalar))

    # --- Operaciones de Álgebra Lineal Clave ---
    def dot(self, other):
        """Producto punto: Fundamental para calcular iluminación y ángulos."""
        return np.dot(self.components, other.components)

    def cross(self, other):
        """Producto cruz: Para hallar perpendiculares (normales)."""
        return Vec3(*np.cross(self.components, other.components))

    def length(self):
        """Magnitud del vector: sqrt(x² + y² + z²)"""
        return np.linalg.norm(self.components)

    def normalize(self):
        """Convierte el vector en unitario (longitud = 1).
           Esencial para direcciones de rayos y luces.
        """
        mag = self.length()
        if mag == 0:
            return Vec3(0, 0, 0)
        return self / mag

    def __repr__(self):
        return f"Vec3({self.x}, {self.y}, {self.z})"

    def reflect(self, normal):
        """Calcula el vector de reflexión especular."""
        return self - normal * (2 * self.dot(normal))