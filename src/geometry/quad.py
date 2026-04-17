"""Cuadrilátero - superficie plana para paredes y luces de área."""

import math
import random
from typing import Optional, Tuple
from src.vector import Vec3
from src.ray import Ray
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB


class Quad:
    """
    Cuadrilátero (paralelogramo) plano en el espacio 3D.

    Definido por un vértice origen Q y dos vectores de lado u, v.
    Cualquier punto en el quad es P = Q + s*u + t*v con s,t ∈ [0,1].

    Atributos:
        Q (Vec3): Vértice origen del quad
        u (Vec3): Vector del primer lado
        v (Vec3): Vector del segundo lado
        material (Material): Material del quad
        normal (Vec3): Normal del plano (u × v normalizado)
        D (float): Constante del plano (normal · Q)
        w (Vec3): Vector auxiliar para coordenadas paramétricas
    """

    def __init__(
        self,
        Q: Vec3,
        u: Vec3,
        v: Vec3,
        material: "Material",
    ):
        """
        Inicializa un cuadrilátero con su geometría y material.

        Durante la inicialización se precalculan varios valores para
        acelerar las pruebas de intersección:
        - normal: u × v normalizado
        - D: distancia del plano al origen (normal · Q)
        - w: vector auxiliar para coordenadas paramétricas

        Args:
            Q: Punto origen del quad (una esquina)
            u: Vector del primer lado
            v: Vector del segundo lado (no necesita ser perpendicular a u)
            material: Material del quad (Material ABC)

        Nota:
            Los vectores u y v NO necesitan ser perpendiculares entre sí,
            lo que permite representar cuadriláteros no rectangulares (paralelogramos).

        Ejemplo:
            >>> from src.materials import DiffuseMaterial
            >>> mat = DiffuseMaterial(Vec3(0.8, 0.3, 0.3))
            >>> quad = Quad(Vec3(0,0,0), Vec3(1,0,0), Vec3(0,1,0), mat)
        """
        self.Q = Q
        self.u = u
        self.v = v
        self.material = material

        # Precalculamos valores para la intersección
        n = u.cross(v)
        self.normal = n.normalize()
        self.D = self.normal.dot(self.Q)
        self.w = n / n.dot(n)  # Vector auxiliar para coordenadas (u, v)

    @property
    def center(self):
        """
        Returns:
            Vec3: Centro geométrico del quad: Q + 0.5*u + 0.5*v
        """
        # El centro de un Quad es el punto de origen Q desplazado
        # a la mitad de sus vectores u y v.
        return self.Q + (self.u * 0.5) + (self.v * 0.5)

    def bounding_box(self):
        """
        Caja envolvente mínima de las 4 esquinas del quad, con margen de 0.0001
        para evitar cajas de espesor cero en quads alineados con un plano.

        Returns:
            AABB: Caja envolvente del quad
        """
        # Calculamos las 4 esquinas
        p1 = self.Q
        p2 = self.Q + self.u
        p3 = self.Q + self.v
        p4 = self.Q + self.u + self.v

        # Buscamos los mínimos y máximos (añadimos un margen de 0.0001 por si es plano)
        min_pt = Vec3(
            min(p1.x, p2.x, p3.x, p4.x) - 0.0001,
            min(p1.y, p2.y, p3.y, p4.y) - 0.0001,
            min(p1.z, p2.z, p3.z, p4.z) - 0.0001,
        )
        max_pt = Vec3(
            max(p1.x, p2.x, p3.x, p4.x) + 0.0001,
            max(p1.y, p2.y, p3.y, p4.y) + 0.0001,
            max(p1.z, p2.z, p3.z, p4.z) + 0.0001,
        )
        return AABB(min_pt, max_pt)

    def hit(self, ray: Ray, t_min=0.001, t_max=float("inf")):
        """
        Intersección rayo-quad en dos fases: rayo-plano, luego test de contención.

        Fase 1: t = (D - n·O) / (n·dir). Si n·dir ≈ 0 el rayo es paralelo al plano.
        Fase 2: Coordenadas baricéntricas alpha, beta. Válido si ambas ∈ [0, 1].

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: Información del impacto, o None si no hay intersección
        """
        denom = self.normal.dot(ray.direction)

        # Si el rayo es paralelo al plano, no hay impacto
        if abs(denom) < 1e-8:
            return None

        # Distancia t hasta el plano
        t = (self.D - self.normal.dot(ray.origin)) / denom
        if t < t_min or t > t_max:
            return None

        # Determinamos si el punto de impacto está dentro de los límites u y v
        intersection = ray.point_at(t)
        planar_hit_pt_vector = intersection - self.Q

        # Usamos el vector w para proyectar el punto en coordenadas alfa y beta
        alpha = self.w.dot(planar_hit_pt_vector.cross(self.v))
        beta = self.w.dot(self.u.cross(planar_hit_pt_vector))

        if not (0 <= alpha <= 1 and 0 <= beta <= 1):
            return None

        return HitRecord(
            t=t,
            point=intersection,
            normal=self.normal,
            material=self.material,
            obj_ref=self,
        )

    def sample_solid_angle(self, hit_point):
        """
        Muestreo de ángulo sólido para importance sampling de luces de área.

        Elige un punto aleatorio en el quad y calcula Ω = (A · |cos_light|) / d²,
        donde A es el área, cos_light el coseno con la normal de la luz y d la distancia.

        Args:
            hit_point (Vec3): Punto desde el cual se muestrea la luz

        Returns:
            tuple: (dirección: Vec3, ángulo_sólido: float) en estereorradianes
        """
        # 1. Elegimos un punto aleatorio en la superficie del Quad
        # Punto = Origen + (u * random) + (v * random)
        random_point = self.Q + (self.u * random.random()) + (self.v * random.random())

        # 2. Calculamos el vector dirección y la distancia
        direction_to_light = random_point - hit_point
        distance_sq = direction_to_light.dot(direction_to_light)
        distance = math.sqrt(distance_sq)
        direction = direction_to_light / distance

        # 3. Calculamos el área del cuadrilátero (magnitud del producto cruz)
        area = self.u.cross(self.v).length()

        # 4. Calculamos el coseno del ángulo entre la normal del Quad y la dirección del rayo
        # Usamos abs porque la luz puede emitir por ambos lados o estar orientada
        cos_light = abs(self.normal.dot(direction))

        # 5. Ángulo sólido (Omega) para un parche plano:
        # Omega = (Area * cos_theta_luz) / distancia^2
        # Esto convierte la probabilidad de área en probabilidad de ángulo sólido
        solid_angle = (area * cos_light) / distance_sq

        return direction, solid_angle
