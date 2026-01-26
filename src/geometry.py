import math
import random
from src.vector import Vec3
from src.ray import Ray


class BVHNode:
    def __init__(self, left, right, box):
        """
        Constructor privado. 
        Se recomienda usar BVHNode.create() para construir el árbol.
        """
        self.left = left
        self.right = right
        self.box = box

    @classmethod
    def create(cls, objects, start=0, end=None):
        """
        Método de fábrica recursivo para construir el árbol BVH.
        Implementa polimorfismo: las hojas pueden ser esferas directas o nodos.
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
        Calcula la intersección más cercana de forma jerárquica.
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
        """Retorna la caja envolvente del nodo."""
        return self.box


class AABB:
    def __init__(self, min_pt: Vec3, max_pt: Vec3):
        self.min = min_pt
        self.max = max_pt

    def hit(self, ray: Ray, t_min, t_max):
        """Algoritmo de intersección de cajas alineadas con los ejes."""
        for i in range(3):
            # Obtenemos las componentes x, y, z
            origin_i = [ray.origin.x, ray.origin.y, ray.origin.z][i]
            direction_i = [ray.direction.x, ray.direction.y, ray.direction.z][i]
            min_i = [self.min.x, self.min.y, self.min.z][i]
            max_i = [self.max.x, self.max.y, self.max.z][i]

            invD = 1.0 / direction_i
            t0 = (min_i - origin_i) * invD
            t1 = (max_i - origin_i) * invD

            if invD < 0:
                t0, t1 = t1, t0

            t_min = max(t0, t_min)
            t_max = min(t1, t_max)

            if t_max <= t_min:
                return False
        return True

    def union(self, other):
        """Crea una nueva caja que envuelve a esta y a otra."""
        new_min = Vec3(
            min(self.min.x, other.min.x),
            min(self.min.y, other.min.y),
            min(self.min.z, other.min.z),
        )
        new_max = Vec3(
            max(self.max.x, other.max.x),
            max(self.max.y, other.max.y),
            max(self.max.z, other.max.z),
        )
        return AABB(new_min, new_max)


class HitRecord:
    def __init__(
        self,
        t,
        point,
        normal,
        color,
        emission,
        is_metal=False,
        fuzz=0.0,
        is_dielectric=False,
        ior=1.5,
        obj_ref=None,
    ):
        self.t = t
        self.point = point
        self.normal = normal
        self.color = color
        self.emission = emission
        self.is_metal = is_metal
        self.fuzz = fuzz
        self.is_dielectric = is_dielectric
        self.ior = ior
        self.obj_ref = obj_ref


class Sphere:
    def __init__(
        self,
        center: Vec3,
        radius: float,
        color: Vec3,
        emission=None,
        is_metal=False,
        fuzz=0.0,
        is_dielectric=False,
        ior=1.5,
    ):
        self.center = center
        self.radius = radius
        self.color = color
        # Si no se define, la esfera no emite luz (Vec3(0,0,0))
        self.emission = emission if emission else Vec3(0, 0, 0)
        self.is_metal = is_metal
        self.fuzz = fuzz if fuzz <= 1.0 else 1.0  # Limitamos a 1.0
        self.is_dielectric = is_dielectric
        self.ior = ior

    def hit(self, ray: Ray, t_min=0.001, t_max=float("inf")):
        """
        Retorna el HitRecord más cercano si hay impacto en el rango [t_min, t_max],
        de lo contrario retorna None.
        """
        oc = ray.origin - self.center

        a = ray.direction.dot(ray.direction)
        b = 2.0 * ray.direction.dot(oc)
        c = oc.dot(oc) - self.radius**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None

        sqrtd = math.sqrt(discriminant)

        # Encontramos la raíz más cercana que esté en el rango aceptable
        root = (-b - sqrtd) / (2.0 * a)
        if root <= t_min or t_max <= root:
            root = (-b + sqrtd) / (2.0 * a)
            if root <= t_min or t_max <= root:
                return None

        t = root
        point = ray.point_at(t)
        # Calculamos la normal y la normalizamos
        normal = (point - self.center) / self.radius

        return HitRecord(
            t,
            point,
            normal,
            self.color,
            self.emission,
            self.is_metal,
            self.fuzz,
            self.is_dielectric,
            self.ior,
            obj_ref=self,
        )

    def random_point_on_surface(self):
        """
        Genera un punto aleatorio en la superficie de la esfera.
        """
        # Generamos una dirección aleatoria uniforme en una esfera
        theta = 2 * math.pi * random.random()
        phi = math.acos(2 * random.random() - 1)

        dx = math.sin(phi) * math.cos(theta)
        dy = math.sin(phi) * math.sin(theta)
        dz = math.cos(phi)

        direction = Vec3(dx, dy, dz)
        # El punto es: centro + (dirección_unitaria * radio)
        return self.center + direction * self.radius

    def sample_solid_angle(self, hit_point):
        """
        Genera una dirección aleatoria dentro del cono que subtiende
        la esfera desde el punto de choque (Muestreo de Cono Sólido).
        """
        direction_to_center = self.center - hit_point
        dist_sq = direction_to_center.dot(direction_to_center)
        dist = math.sqrt(dist_sq)

        # Dirección normalizada hacia el centro de la luz
        z_axis = direction_to_center.normalize()

        # Calculamos el ángulo máximo del cono (sin(theta) = R/d)
        # Si estamos dentro de la luz, el radio es 1 (toda la esfera)
        sin_theta_max_sq = (self.radius * self.radius) / dist_sq
        cos_theta_max = math.sqrt(max(0, 1 - sin_theta_max_sq))

        # Muestreo aleatorio dentro del cono
        r1 = random.random()
        r2 = random.random()

        phi = 2 * math.pi * r1
        # cos_theta varía entre 1 (centro) y cos_theta_max (borde del cono)
        cos_theta = 1 - r2 * (1 - cos_theta_max)
        sin_theta = math.sqrt(max(0, 1 - cos_theta * cos_theta))

        # Dirección en espacio local del cono
        local_dir = Vec3(
            math.cos(phi) * sin_theta, math.sin(phi) * sin_theta, cos_theta
        )

        # Transformar a espacio global (creando una base ortonormal simple)
        # Buscamos un vector no paralelo a z_axis
        helper = Vec3(1, 0, 0) if abs(z_axis.x) < 0.8 else Vec3(0, 1, 0)
        x_axis = z_axis.cross(helper).normalize()
        y_axis = z_axis.cross(x_axis)

        world_dir = x_axis * local_dir.x + y_axis * local_dir.y + z_axis * local_dir.z

        # El Ángulo Sólido (Omega) es la "cantidad de cielo" que ocupa la luz
        solid_angle = 2 * math.pi * (1 - cos_theta_max)

        return world_dir.normalize(), solid_angle

    def bounding_box(self):
        return AABB(
            self.center - Vec3(self.radius, self.radius, self.radius),
            self.center + Vec3(self.radius, self.radius, self.radius),
        )
