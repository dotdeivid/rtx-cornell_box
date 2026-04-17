"""BVH (Bounding Volume Hierarchy) para aceleración espacial."""

import random
from src.ray import Ray
from src.geometry.aabb import AABB
from src.geometry.hit_record import HitRecord

class BVHNode:
    """
    Árbol binario de jerarquía de volúmenes envolventes (BVH).

    Reduce la complejidad de intersección de O(n) a O(log n) mediante poda espacial.
    Cada nodo almacena una AABB que contiene todos sus descendientes.

    Atributos:
        left: Hijo izquierdo (BVHNode u objeto geométrico)
        right: Hijo derecho (BVHNode u objeto geométrico)
        box (AABB): Caja envolvente que contiene ambos hijos
    """

    def __init__(self, left, right, box):
        """
        Constructor privado del nodo BVH.

        No se recomienda usar este constructor directamente. En su lugar,
        utiliza el método de fábrica BVHNode.create() que construye el árbol
        completo automáticamente de forma óptima.

        Args:
            left: Hijo izquierdo (BVHNode u objeto geométrico)
            right: Hijo derecho (BVHNode u objeto geométrico)
            box (AABB): Caja envolvente que contiene a ambos hijos
        """
        self.left = left
        self.right = right
        self.box = box

    @classmethod
    def create(cls, objects, start=0, end=None):
        """
        Construye el árbol BVH recursivamente dividiendo por eje aleatorio.

        Ordena objetos por su AABB en un eje aleatorio y divide a la mitad.
        Con 1 objeto retorna el objeto directamente; con 2, crea un nodo hoja.

        Args:
            objects (list): Lista de objetos geométricos (deben implementar hit() y bounding_box())
            start (int): Índice inicial del segmento
            end (int): Índice final (exclusivo). Por defecto len(objects)

        Returns:
            BVHNode o objeto geométrico: Raíz del subárbol construido
        """
        if end is None:
            end = len(objects)

        # 1. Elegir un eje aleatorio para dividir el espacio (X=0, Y=1, Z=2)
        axis = random.randint(0, 2)

        # Función auxiliar para comparar las cajas envolventes en el eje elegido
        def box_compare(obj):
            box = obj.bounding_box()
            # Retorna el punto mínimo en el eje seleccionado
            return [box.min.x, box.min.y, box.min.z][axis]

        span = end - start

        # CASO BASE: Solo un objeto
        if span == 1:
            # OPTIMIZACIÓN: Retornamos el objeto directamente (ej. Sphere)
            # Esto evita crear un nodo innecesario que apunte a sí mismo.
            return objects[start]

        # CASO BASE: Dos objetos
        elif span == 2:
            # Ordenamos los dos objetos para que el árbol sea predecible
            if box_compare(objects[start]) < box_compare(objects[start + 1]):
                left, right = objects[start], objects[start + 1]
            else:
                left, right = objects[start + 1], objects[start]

        # CASO RECURSIVO: Más de dos objetos
        else:
            # Ordenamos el segmento actual de la lista basado en el eje
            segment = objects[start:end]
            segment.sort(key=box_compare)
            objects[start:end] = segment

            # Punto de división media
            mid = start + span // 2

            # Construcción recursiva de las ramas
            left = cls.create(objects, start, mid)
            right = cls.create(objects, mid, end)

        # Creamos la caja envolvente (AABB) total que cubre a ambos hijos
        # Tanto 'left' como 'right' pueden ser un BVHNode o una Sphere
        full_box = left.bounding_box().union(right.bounding_box())

        return cls(left, right, full_box)

    def hit(self, ray, t_min, t_max):
        """
        Intersección jerárquica: poda el subárbol si el rayo no toca la AABB del nodo.

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida (normalmente 0.001)
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: La intersección más cercana en [t_min, t_max], o None
        """
        # PODA (Pruning): Si el rayo no toca la caja contenedora,
        # ignoramos todo lo que hay dentro. Ahorro masivo de CPU.
        if not self.box.hit(ray, t_min, t_max):
            return None

        # Buscamos en el hijo izquierdo primero
        hit_left = self.left.hit(ray, t_min, t_max)

        # OPTIMIZACIÓN: Si golpeamos algo a la izquierda,
        # actualizamos el límite de búsqueda para el hijo derecho.
        # No nos interesa nada que esté más lejos que hit_left.t
        limit = hit_left.t if hit_left else t_max
        hit_right = self.right.hit(ray, t_min, limit)

        # Retornamos el impacto que haya ocurrido más cerca de la cámara
        return hit_right if hit_right else hit_left

    def bounding_box(self):
        """
        Returns:
            AABB: Caja envolvente precalculada que contiene todos los descendientes
        """
        return self.box
