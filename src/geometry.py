import math
import random
from src.vector import Vec3
from src.ray import Ray


class BVHNode:
    """
    Nodo de la Jerarquía de Volúmenes Envolventes (Bounding Volume Hierarchy - BVH).

    El BVH es una estructura de datos de árbol binario que organiza objetos geométricos
    en el espacio 3D para acelerar las pruebas de intersección rayo-objeto.

    Estructura:
    - Cada nodo tiene dos hijos (left, right) que pueden ser:
      * Otros BVHNode (nodos internos)
      * Objetos geométricos directos como Sphere, Quad, Triangle (hojas)
    - Cada nodo tiene una AABB (caja envolvente) que contiene todos sus descendientes

    Beneficio:
    - Reduce la complejidad de O(n) a O(log n) para intersecciones de rayos
    - Implementa "poda espacial": si un rayo no intersecta la caja padre,
      podemos descartar todos los objetos contenidos sin probarlos individualmente

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
        Método de fábrica recursivo para construir el árbol BVH de forma óptima.

        Este método implementa la construcción del BVH usando el algoritmo SAH
        (Surface Area Heuristic) simplificado con división por eje aleatorio.

        Algoritmo:
        1. Selecciona un eje aleatorio (X, Y o Z) para dividir el espacio
        2. Ordena los objetos según la posición de sus cajas en ese eje
        3. Divide la lista por la mitad y construye recursivamente subárboles
        4. Crea una caja envolvente que contiene ambos subárboles

        Casos base:
        - 1 objeto: Retorna el objeto directamente (optimización, evita nodo innecesario)
        - 2 objetos: Crea un nodo con ambos objetos como hijos directos

        Polimorfismo:
        Las hojas pueden ser objetos geométricos (Sphere, Quad, Triangle) o nodos BVH.
        Todos implementan los métodos hit() y bounding_box() con la misma interfaz.

        Args:
            objects (list): Lista de objetos geométricos a organizar en el BVH
            start (int, opcional): Índice inicial del segmento a procesar. Por defecto 0.
            end (int, opcional): Índice final (exclusivo) del segmento. Por defecto len(objects).

        Returns:
            BVHNode o objeto geométrico: La raíz del (sub)árbol BVH construido

        Ejemplo:
            >>> spheres = [Sphere(...), Sphere(...), Sphere(...)]
            >>> bvh_root = BVHNode.create(spheres)
            >>> # Ahora puedes usar bvh_root.hit(ray, 0.001, inf) para buscar intersecciones
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
        Calcula la intersección del rayo con los objetos contenidos usando búsqueda jerárquica.

        Este método implementa el algoritmo de traversal del BVH que aprovecha la
        jerarquía espacial para acelerar las pruebas de intersección.

        Algoritmo de Poda Espacial (Spatial Pruning):
        1. Primero prueba si el rayo intersecta la caja envolvente del nodo
        2. Si NO intersecta → retorna None (descarta TODO el subárbol sin más pruebas)
        3. Si SÍ intersecta → prueba ambos hijos (left y right)
        4. Durante la búsqueda en el hijo derecho, optimiza el rango usando el hit izquierdo

        Optimización de rango:
        - Si encontramos un hit en el hijo izquierdo a distancia t_left,
          no necesitamos buscar hits en el hijo derecho más allá de t_left
        - Esto reduce significativamente el número de pruebas

        Args:
            ray (Ray): El rayo con el que probar la intersección
            t_min (float): Distancia mínima válida para considerar una intersección.
                          Típicamente 0.001 para evitar "acné de sombra" (shadow acne)
            t_max (float): Distancia máxima válida. Típicamente float('inf')

        Returns:
            HitRecord o None: El registro de la intersección más cercana encontrada
                             dentro del rango [t_min, t_max], o None si no hay intersección

        Eficiencia:
        - Sin BVH: O(n) - debe probar todos los objetos
        - Con BVH: O(log n) promedio - descarta subárboles completos
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
        Retorna la caja envolvente (AABB) que contiene este nodo y todos sus descendientes.

        Esta caja fue precalculada durante la construcción del BVH y representa
        el volumen mínimo alineado con los ejes que contiene completamente
        ambos hijos (left y right).

        Returns:
            AABB: La caja envolvente del nodo
        """
        return self.box


class AABB:
    """
    Axis-Aligned Bounding Box (Caja Envolvente Alineada con los Ejes).

    Una AABB es el volumen rectangular mínimo alineado con los ejes X, Y, Z
    que contiene completamente un objeto o conjunto de objetos 3D.

    "Alineada con los ejes" significa que las caras de la caja son siempre
    perpendiculares a los ejes X, Y, Z. Esto simplifica enormemente los cálculos
    de intersección comparado con cajas orientadas arbitrariamente (OBB).

    Usos:
    - Aceleración de ray tracing mediante pruebas rápidas de intersección
    - Construcción de estructuras jerárquicas como BVH
    - Detección de colisiones en motores de física

    Ventajas sobre probar la geometría completa directamente:
    - Test de intersección MUCHO más rápido (solo 6 comparaciones vs ecuaciones cuadráticas)
    - Simplicidad computacional: solo requiere min/max

    Atributos:
        min (Vec3): Punto con las coordenadas mínimas (x_min, y_min, z_min)
        max (Vec3): Punto con las coordenadas máximas (x_max, y_max, z_max)

    Ejemplo:
        >>> # Caja que va de (0,0,0) a (2,3,4)
        >>> box = AABB(Vec3(0,0,0), Vec3(2,3,4))
    """

    def __init__(self, min_pt: Vec3, max_pt: Vec3):
        """
        Inicializa una caja envolvente alineada con los ejes.

        Args:
            min_pt (Vec3): Esquina con las coordenadas mínimas en cada eje
            max_pt (Vec3): Esquina con las coordenadas máximas en cada eje

        Nota:
            Se asume que min_pt.x <= max_pt.x, min_pt.y <= max_pt.y, min_pt.z <= max_pt.z
        """
        self.min = min_pt
        self.max = max_pt

    def hit(self, ray: Ray, t_min, t_max):
        """
        Prueba si un rayo intersecta esta caja envolvente usando el algoritmo de slabs.

        Algoritmo de Slabs (Kay y Kajiya, 1986):
        La idea es ver la caja como la intersección de 3 pares de planos paralelos
        (slabs) perpendiculares a los ejes X, Y, Z.

        Para cada eje:
        1. Calculamos dónde el rayo entra al slab (t0) y dónde sale (t1)
        2. Intersectamos estos intervalos [t0, t1] para los 3 ejes
        3. Si hay una intersección no vacía → el rayo toca la caja

        Optimización:
        - Usamos la inversa de la dirección (1/d) para evitar divisiones en el bucle
        - Intercambiamos t0 y t1 si la dirección es negativa (invD < 0)
        - Hacemos early-out: si en cualquier eje el intervalo se vuelve imposible,
          retornamos false inmediatamente

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Parámetro mínimo del rayo (normalmente 0.001)
            t_max (float): Parámetro máximo del rayo (normalmente infinito)

        Returns:
            bool: True si el rayo intersecta la caja en el rango [t_min, t_max]

        Complejidad:
            O(1) - siempre 3 iteraciones independiente del tamaño de la escena

        Nota sobre valores:
            - t_min evita el "acné de sombra" (shadow acne) por errores numéricos
            - t_max limita la distancia máxima de búsqueda
        """
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
        """
        Crea una nueva AABB que envuelve completamente esta caja y otra caja.

        Esta operación es fundamental para la construcción del BVH: cuando
        combinamos dos nodos hijos, necesitamos una caja padre que los contenga a ambos.

        Algoritmo:
        - Para cada eje (x, y, z): toma el mínimo de ambos mínimos y el máximo de ambos máximos

        Args:
            other (AABB): La otra caja envolvente a unir

        Returns:
            AABB: Nueva caja que es la unión de ambas cajas

        Ejemplo:
            >>> box1 = AABB(Vec3(0,0,0), Vec3(1,1,1))
            >>> box2 = AABB(Vec3(0.5,0.5,0.5), Vec3(2,2,2))
            >>> combined = box1.union(box2)
            >>> # combined abarca de (0,0,0) a (2,2,2)

        Propiedades matemáticas:
        - Conmutativa: box1.union(box2) == box2.union(box1)
        - La caja resultante siempre es >= a ambas cajas originales en volumen
        """
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
    """
    Registro de información sobre una intersección rayo-objeto.

    Cuando un rayo impacta un objeto geométrico, este registro almacena
    toda la información necesaria para calcular la iluminación y el rebote
    del rayo en ese punto.

    Atributos:
        t (float): Parámetro del rayo donde ocurre la intersección.
                   El punto de impacto es: ray.origin + t * ray.direction
                   Valores menores de t = objetos más cercanos a la cámara

        point (Vec3): Punto exacto 3D donde el rayo intersectó la superficie

        normal (Vec3): Vector normal a la superficie en el punto de impacto (normalizado).
                       Siempre apunta "hacia afuera" del objeto

        color (Vec3): Color base del material (albedo) en formato RGB [0-1]

        emission (Vec3): Luz emitida por el objeto en ese punto (para fuentes de luz)
                        Vec3(0,0,0) = no emite luz
                        Vec3(15,15,15) = luz blanca muy brillante

        is_metal (bool): True si el material es metálico (reflectante)

        fuzz (float): Rugosidad del metal [0-1]
                     0.0 = espejo perfecto (reflexión especular pura)
                     1.0 = metal muy rugoso (reflexiones difusas)
                     Valores mayores se limitan a 1.0

        is_dielectric (bool): True si el material es dieléctrico (vidrio, agua, etc.)
                             Los dieléctricos refractan la luz además de reflejarla

        ior (float): Índice de refracción (Index of Refraction)
                    1.0 = vacío/aire
                    1.33 = agua
                    1.5 = vidrio común
                    2.4 = diamante
                    Controla cuánto se dobla la luz al atravesar el material

        obj_ref: Referencia al objeto geométrico que fue impactado
                (para acceder a propiedades específicas si es necesario)
    """

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
        """
        Inicializa un registro de intersección rayo-superficie.

        Args:
            t (float): Distancia desde el origen del rayo hasta el punto de impacto
            point (Vec3): Coordenadas 3D del punto de intersección
            normal (Vec3): Normal de la superficie (debe estar normalizada)
            color (Vec3): Color del material (albedo)
            emission (Vec3): Luz emitida por la superficie
            is_metal (bool): Si el material es metálico
            fuzz (float): Rugosidad para metales [0-1]
            is_dielectric (bool): Si el material es transparente/refractivo
            ior (float): Índice de refracción
            obj_ref: Referencia al objeto que generó este hit
        """
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
    """
    Representa una esfera en el espacio 3D con propiedades de material.

    Una esfera es la superficie de todos los puntos que están a una distancia
    constante (radio) de un punto central. Es la primitiva geométrica más
    simple y común en ray tracing.

    Ecuación matemática:
        ||P - C||^2 = r^2
        donde P es cualquier punto en la superficie, C es el centro, r es el radio

    Atributos:
        center (Vec3): Centro de la esfera en coordenadas 3D
        radius (float): Radio de la esfera (debe ser > 0)
        color (Vec3): Color del material (albedo) en RGB [0-1]
        emission (Vec3): Luz emitida (para esferas luminosas)
        is_metal (bool): Si es metálico (reflectante)
        fuzz (float): Rugosidad del metal [0-1], limitado a 1.0
        is_dielectric (bool): Si es transparente/refractivo (vidrio, agua, etc.)
        ior (float): Índice de refracción (1.0=aire, 1.5=vidrio)
    """

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
        """
        Inicializa una esfera con su geometría y propiedades de material.

        Args:
            center (Vec3): Posición del centro de la esfera
            radius (float): Radio de la esfera
            color (Vec3): Color base del material (valores 0-1 para R,G,B)
            emission (Vec3, opcional): Luz emitida. Por defecto Vec3(0,0,0) (no emite)
            is_metal (bool): True para materiales metálicos reflectantes
            fuzz (float): Rugosidad del metal [0-1]. 0=espejo perfecto, 1=muy rugoso
            is_dielectric (bool): True para materiales transparentes
            ior (float): Índice de refracción para dieléctricos

        Nota:
            El parámetro fuzz se limita automáticamente a 1.0 si se pasa un valor mayor
        """
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
        Calcula la intersección de un rayo con esta esfera usando la ecuación cuadrática.

        Derivación matemática:
        Un rayo se parametriza como: P(t) = O + t*D  (O=origen, D=dirección, t=parámetro)
        Una esfera se define como: ||P - C||^2 = r^2  (C=centro, r=radio)

        Sustituyendo:
            ||O + t*D - C||^2 = r^2
        Expandiendo:
            (O + t*D - C) · (O + t*D - C) = r^2
        Sea oc = O - C:
            (oc + t*D) · (oc + t*D) = r^2
            D·D * t^2 + 2*D·oc * t + oc·oc - r^2 = 0

        Esto es una ecuación cuadrática: at^2 + bt + c = 0
        donde:
            a = D · D  (magnitud al cuadrado de la dirección)
            b = 2 * D · oc
            c = oc · oc - r^2

        Solución:
            t = (-b ± sqrt(b^2 - 4ac)) / 2a

        El discriminante (b^2 - 4ac) determina:
            < 0: No hay intersección (el rayo pasa de largo)
            = 0: Intersección tangente (roza la esfera)
            > 0: Dos intersecciones (entra y sale de la esfera)

        Elegimos la raíz MENOR (entrada) que esté en el rango [t_min, t_max].
        Si no es válida, probamos la raíz MAYOR (salida).

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Parámetro mínimo válido (por defecto 0.001 para evitar acné)
            t_max (float): Parámetro máximo válido (por defecto infinito)

        Returns:
            HitRecord o None: Información del impacto si hay intersección válida,
                             None en caso contrario

        Nota sobre t_min=0.001:
            Evita el "acné de sombra" (shadow acne). Debido a errores de punto flotante,
            un rayo que rebota en una superficie podría re-intersectar la misma superficie
            en t ≈ 0. Usar t_min=0.001 previene esto.
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
        Genera un punto aleatorio uniformemente distribuido en la superficie de la esfera.

        Este método usa muestreo esférico en coordenadas polares para garantizar
        una distribución uniforme.

        Algoritmo:
        1. Genera ángulos aleatorios theta (azimut) y phi (polar)
        2. Convierte de coordenadas esféricas a cartesianas
        3. Escala por el radio y desplaza al centro

        Matemáticas:
            theta ~ Uniforme(0, 2π)  # Ángulo azimutal
            phi ~ arccos(2*u - 1) donde u ~ Uniforme(0,1)  # Ángulo polar

            Conversión a cartesianas:
            x = sin(phi) * cos(theta)
            y = sin(phi) * sin(theta)
            z = cos(phi)

            Punto final = centro + radio * dirección_unitaria

        Por qué phi = arccos(2*u - 1)?
            Si usamos phi = u*pi directamente, habría más puntos cerca de los polos.
            La distribución arccos(2*u - 1) compensa la menor circunferencia en los polos,
            garantizando uniformidad en el área superficial.

        Returns:
            Vec3: Punto aleatorio en la superficie de la esfera

        Uso:
            Típicamente usado para muestreo de luces esféricas en path tracing
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
        Muestreo por Ángulo Sólido (Solid Angle Sampling) para Importance Sampling.

        En lugar de muestrear puntos aleatorios en toda la esfera, este método
        genera direcciones aleatorias dentro del CONO que subtiende la esfera
        vista desde hit_point.

        ¿Por qué es mejor?
        - Muestreo uniforme desperdiciaría rayos en direcciones que no llegan a la luz
        - El muestreo de cono concentra los rayos donde realmente pueden contribuir luz
        - Reduce significativamente el ruido en el renderizado

        Conceptos:

        1. Ángulo Sólido (Ω):
           Es la "cantidad de cielo" que ocupa un objeto visto desde un punto.
           Medido en estereorradianes (sr). Una esfera completa = 4π sr.

           Para una esfera de radio R a distancia d:
           Ω = 2π(1 - cos(θ_max))
           donde sin(θ_max) = R/d

        2. Ángulo del Cono (θ_max):
           Es el ángulo máximo desde el eje del cono hasta su borde.
           Se calcula usando: sin(θ_max) = radio_esfera / distancia
           Si estamos muy cerca: θ_max ≈ 90° (casi hemisferio)
           Si estamos muy lejos: θ_max ≈ 0° (casi un punto)

        Algoritmo:
        1. Calcula la dirección hacia el centro de la luz (eje Z local)
        2. Calcula θ_max usando sin^2(θ) = R^2/d^2
        3. Genera dirección aleatoria dentro del cono:
           - phi ~ Uniforme(0, 2π)  # Rotación azimutal
           - cos(θ) ~ Uniforme(cos(θ_max), 1)  # Distribución radial
        4. Transforma de coordenadas locales del cono a coordenadas mundo

        Args:
            hit_point (Vec3): Punto desde el cual se muestrea la esfera luminosa

        Returns:
            tuple: (dirección_normalizada, ángulo_sólido)
                - dirección (Vec3): Vector unitario apuntando hacia la esfera
                - solid_angle (float): Ángulo sólido en estereorradianes

        Efecto de la distancia:
        - Cerca de la luz: Ω grande → cono ancho → más variación en direcciones
        - Lejos de la luz: Ω pequeño → cono estrecho → direcciones casi paralelas

        Relación con Importance Sampling:
        El ángulo sólido se usa para corregir la probabilidad en la ecuación de renderizado,
        garantizando que el estimador Monte Carlo sea insesgado.
        """
        direction_to_center = self.center - hit_point
        dist_sq = direction_to_center.dot(direction_to_center)

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
        """
        Calcula la AABB (caja envolvente) mínima que contiene completamente la esfera.

        Para una esfera, la caja envolvente es muy simple:
        - min = centro - (radio, radio, radio)
        - max = centro + (radio, radio, radio)

        Esto crea un cubo alineado con los ejes que toca la esfera en 6 puntos
        (los extremos en cada eje).

        Returns:
            AABB: Caja envolvente de la esfera

        Nota:
            Aunque la caja contiene bastante espacio vacío (las esquinas),
            la simplicidad del cálculo y la prueba de intersección compensa
            esta ineficiencia de espacio.
        """
        return AABB(
            self.center - Vec3(self.radius, self.radius, self.radius),
            self.center + Vec3(self.radius, self.radius, self.radius),
        )


class Quad:
    """
    Representa un cuadrilátero (quad) plano en el espacio 3D.

    Un Quad es un polígono de 4 vértices definido por un punto origen Q
    y dos vectores u y v que definen sus lados. Es muy usado para
    representar superficies planas como paredes, pisos, luces de área, etc.

    Definición paramétrica:
        Cualquier punto P en el quad se puede expresar como:
        P(s, t) = Q + s*u + t*v
        donde s, t ∈ [0, 1]

    Los 4 vértices son:
        - Q (origen)
        - Q + u
        - Q + v
        - Q + u + v

    Atributos:
        Q (Vec3): Punto origen (esquina del cuadrilátero)
        u (Vec3): Vector que define el primer lado
        v (Vec3): Vector que define el segundo lado
        color (Vec3): Color del material
        emission (Vec3): Luz emitida (importante para luces de área)
        is_metal (bool): Si el material es metálico
        fuzz (float): Rugosidad del metal [0-1]
        is_dielectric (bool): Si es transparente
        ior (float): Índice de refracción
        normal (Vec3): Normal del plano (calculada como u × v normalizado)
        D (float): Parámetro D de la ecuación del plano n·P = D
        w (Vec3): Vector auxiliar precalculado para cálculos de coordenadas
    """

    def __init__(
        self,
        Q: Vec3,
        u: Vec3,
        v: Vec3,
        color: Vec3,
        emission=None,
        is_metal=False,
        fuzz=0.0,
        is_dielectric=False,
        ior=1.5,
    ):
        """
        Inicializa un cuadrilátero con su geometría y propiedades de material.

        Durante la inicialización se precalculan varios valores para
        acelerar las pruebas de intersección:
        - normal: u × v normalizado
        - D: distancia del plano al origen (normal · Q)
        - w: vector auxiliar para coordenadas paramétricas

        Args:
            Q (Vec3): Punto origen del quad (una esquina)
            u (Vec3): Vector del primer lado
            v (Vec3): Vector del segundo lado (no necesita ser perpendicular a u)
            color (Vec3): Color del material
            emission (Vec3, opcional): Luz emitida (para luces de área)
            is_metal (bool): Si es metálico
            fuzz (float): Rugosidad [0-1]
            is_dielectric (bool): Si es transparente
            ior (float): Índice de refracción

        Nota:
            Los vectores u y v NO necesitan ser perpendiculares entre sí,
            lo que permite representar cuadriláteros no rectangulares (paralelogramos).
        """
        self.Q = Q
        self.u = u
        self.v = v
        self.color = color
        self.emission = emission if emission else Vec3(0, 0, 0)
        self.is_metal = is_metal
        self.fuzz = fuzz
        self.is_dielectric = is_dielectric
        self.ior = ior

        # Precalculamos valores para la intersección
        n = u.cross(v)
        self.normal = n.normalize()
        self.D = self.normal.dot(self.Q)
        self.w = n / n.dot(n)  # Vector auxiliar para coordenadas (u, v)

    @property
    def center(self):
        """
        Calcula el centro geométrico del cuadrilátero.

        El centro se encuentra en el punto medio del quad, que es:
        centro = Q + 0.5*u + 0.5*v

        Esto corresponde a s=0.5, t=0.5 en la parametrización del quad.

        Returns:
            Vec3: Coordenadas del centro del quad

        Uso:
            Útil para cálculos de iluminación, orientación de cámara, etc.
        """
        # El centro de un Quad es el punto de origen Q desplazado
        # a la mitad de sus vectores u y v.
        return self.Q + (self.u * 0.5) + (self.v * 0.5)

    def bounding_box(self):
        """
        Calcula la AABB que envuelve al cuadrilátero.

        Algoritmo:
        1. Calcula las 4 esquinas del quad:
           - Q, Q+u, Q+v, Q+u+v
        2. Para cada eje (x, y, z), encuentra el mínimo y máximo
        3. Añade un pequeño margen (0.0001) en cada dirección

        El margen es importante porque:
        - Si el quad es perfectamente alineado con un plano (ej. XY),
          tendría espesor 0 en el eje Z
        - Una caja de espesor 0 puede causar problemas numéricos en
          el algoritmo de intersección AABB
        - El margen garantiza una caja con volumen positivo en todas las dimensiones

        Returns:
            AABB: Caja envolvente del quad

        Nota:
            Para un quad perfectamente alineado (ej. pared vertical),
            la caja será muy delgada en una dirección (solo 0.0002 de grosor).
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
        Calcula la intersección del rayo con el cuadrilátero.

        Algoritmo en dos fases:

        FASE 1: Intersección Rayo-Plano
        El quad está contenido en un plano infinito definido por:
            n · P = D
        donde n es la normal y D = n · Q

        Un rayo es: R(t) = O + t*dir
        Sustituyendo en la ecuación del plano:
            n · (O + t*dir) = D
            n · O + t*(n · dir) = D
            t = (D - n · O) / (n · dir)

        Si n · dir ≈ 0 → el rayo es paralelo al plano (no hay intersección)

        FASE 2: Prueba de Contención (¿Está dentro del Quad?)
        Una vez encontrado el punto de intersección P en el plano,
        necesitamos verificar si P está dentro de los límites del quad.

        Usamos coordenadas bariocéntricas (alpha, beta):
            P = Q + alpha*u + beta*v

        Si 0 ≤ alpha ≤ 1 AND 0 ≤ beta ≤ 1 → P está dentro del quad

        Cálculo de alpha y beta:
            alpha = w · ((P-Q) × v)
            beta = w · (u × (P-Q))
        donde w = (u × v) / |u × v|^2  (precalculado en __init__)

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: Información del impacto si hay intersección válida

        Casos especiales:
        - Si denom ≈ 0: rayo paralelo al plano → None
        - Si t fuera de rango [t_min, t_max] → None
        - Si alpha o beta fuera de [0,1] → None (fuera del quad)
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
            t,
            intersection,
            self.normal,
            self.color,
            self.emission,
            self.is_metal,
            self.fuzz,
            self.is_dielectric,
            self.ior,
            obj_ref=self,
        )

    def sample_solid_angle(self, hit_point):
        """
        Muestreo de Ángulo Sólido para luces de área rectangulares/cuadradas.

        Este método es fundamental para Importance Sampling de luces de área.
        En lugar de muestrear direcciones aleatorias en todo el hemisferio,
        concentramos las muestras hacia la luz.

        Algoritmo:

        1. Selección de Punto Aleatorio:
           Elegimos un punto uniforme en el quad usando:
           P = Q + r1*u + r2*v
           donde r1, r2 ~ Uniforme(0,1)

        2. Cálculo de Dirección:
           direction = (P - hit_point).normalize()

        3. Cálculo del Ángulo Sólido:
           Para una superficie plana, el ángulo sólido depende de:
           - Área de la superficie (A)
           - Distancia al punto (d)
           - Ángulo entre la normal de la luz y la dirección (cos_light)

           Ω = (A * cos_light) / d^2

        Interpretación física:
        - cos_light corrige la proyección: una luz vista de lado ocupa menos "cielo"
        - d^2 en el denominador: luces más lejanas ocupan menos ángulo sólido
        - A en el numerador: luces más grandes ocupan más ángulo sólido

        Por qué usamos abs(cos_light)?
        - Permite que la luz emita por ambas caras del quad
        - Útil para paneles de luz delgados visibles desde ambos lados

        Args:
            hit_point (Vec3): Punto desde el cual se muestrea la luz

        Returns:
            tuple: (dirección, ángulo_sólido)
                - dirección (Vec3): Vector unitario hacia un punto aleatorio del quad
                - ángulo_sólido (float): Ángulo sólido subtendido [estereorradianes]

        Efecto de la orientación:
        - Luz de frente (cos ≈ 1): ángulo sólido máximo
        - Luz de lado (cos ≈ 0): ángulo sólido pequeño
        - Luz de espaldas: abs() permite contribución desde el otro lado

        Aplicación en Path Tracing:
        Este valor se usa para corregir el peso de la muestra:
        contribución = BRDF * emisión * cos_surface / (pdf / solid_angle)
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


class Triangle:
    """
    Representa un triángulo en el espacio 3D.

    El triángulo es la primitiva geométrica más fundamental en gráficos 3D.
    Se define por tres vértices ordenados (v0, v1, v2) que determinan
    un plano y un área finita.

    Definición paramétrica (Coordenadas Baricéntricas):
        Cualquier punto P en el triángulo se puede expresar como:
        P = v0 + u*(v1-v0) + v*(v2-v0)
        donde u ≥ 0, v ≥ 0, u+v ≤ 1

    La normal del triángulo se calcula como:
        normal = (v1-v0) × (v2-v0) normalizado

    Usos:
    - Meshes 3D (modelos complejos se componen de muchos triángulos)
    - Rasterización (GPUs renderizan todo como triángulos)
    - Ray tracing de geometría compleja

    Atributos:
        v0, v1, v2 (Vec3): Los tres vértices del triángulo (ordenados)
        color (Vec3): Color del material
        emission (Vec3): Luz emitida
        is_metal (bool): Si es metálico
        fuzz (float): Rugosidad [0-1]
        is_dielectric (bool): Si es transparente
        ior (float): Índice de refracción
        normal (Vec3): Normal precalculada del triángulo
    """

    def __init__(
        self,
        v0: Vec3,
        v1: Vec3,
        v2: Vec3,
        color: Vec3,
        emission=None,
        is_metal=False,
        fuzz=0.0,
        is_dielectric=False,
        ior=1.5,
    ):
        """
        Inicializa un triángulo con sus tres vértices y propiedades de material.

        Durante la inicialización se precalcula la normal usando el producto cruz:
            normal = (v1-v0) × (v2-v0) normalizado

        La dirección de la normal sigue la regla de la mano derecha:
        si los vértices están ordenados en sentido antihorario (cuando
        se ven de frente), la normal apunta hacia el observador.

        Args:
            v0 (Vec3): Primer vértice
            v1 (Vec3): Segundo vértice
            v2 (Vec3): Tercer vértice
            color (Vec3): Color del material
            emission (Vec3, opcional): Luz emitida
            is_metal (bool): Si es metálico
            fuzz (float): Rugosidad [0-1]
            is_dielectric (bool): Si es transparente
            ior (float): Índice de refracción

        Nota:
            El orden de los vértices importa para determinar qué lado
            del triángulo es "frontal" vs "trasero".
        """
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.color = color
        self.emission = emission if emission else Vec3(0, 0, 0)
        self.is_metal = is_metal
        self.fuzz = fuzz
        self.is_dielectric = is_dielectric
        self.ior = ior

        # Precalculamos la normal del triángulo
        edge1 = self.v1 - self.v0
        edge2 = self.v2 - self.v0
        self.normal = edge1.cross(edge2).normalize()

    def bounding_box(self):
        """
        Calcula la AABB que envuelve al triángulo.

        Algoritmo:
        1. Para cada eje (x, y, z), encuentra el mínimo y máximo entre los 3 vértices
        2. Añade un pequeño margen (0.001) para evitar cajas de espesor 0

        El margen es necesario por las mismas razones que en Quad:
        - Un triángulo perfectamente alineado tendría espesor 0 en una dirección
        - Esto podría causar problemas numéricos en el cálculo de intersección AABB

        Returns:
            AABB: Caja envolvente del triángulo

        Nota:
            El margen es pequeño (0.001) para minimizar espacio desperdiciado,
            pero suficiente para evitar errores de punto flotante.
        """
        min_pt = Vec3(
            min(self.v0.x, self.v1.x, self.v2.x) - 0.001,
            min(self.v0.y, self.v1.y, self.v2.y) - 0.001,
            min(self.v0.z, self.v1.z, self.v2.z) - 0.001,
        )
        max_pt = Vec3(
            max(self.v0.x, self.v1.x, self.v2.x) + 0.001,
            max(self.v0.y, self.v1.y, self.v2.y) + 0.001,
            max(self.v0.z, self.v1.z, self.v2.z) + 0.001,
        )
        return AABB(min_pt, max_pt)

    def hit(self, ray: Ray, t_min=0.001, t_max=float("inf")):
        """
        Calcula la intersección rayo-triángulo usando el algoritmo Möller-Trumbore.

        El Algoritmo de Möller-Trumbore (1997) es el método estándar de la industria
        para intersección Ray-Triangle porque:
        - Es muy eficiente (no requiere precalcular el plano)
        - Calcula simultáneamente t y las coordenadas baricéntricas
        - Evita divisiones costosas hasta el final

        Matemáticas:

        Un rayo es: R(t) = O + t*D  (O=origen, D=dirección)
        Un punto en el triángulo es: T(u,v) = v0 + u*edge1 + v*edge2
        donde edge1 = v1-v0, edge2 = v2-v0

        En la intersección: R(t) = T(u,v)
            O + t*D = v0 + u*edge1 + v*edge2

        Reordenando:
            O - v0 = -t*D + u*edge1 + v*edge2

        Esto es un sistema de ecuaciones lineales que se resuelve usando
        la regla de Cramer con determinantes y productos cruz.

        Variables del algoritmo:
        - h = D × edge2  (usado para calcular u)
        - a = edge1 · h  (determinante, si ≈0 → rayo paralelo)
        - s = O - v0  (vector desde v0 al origen del rayo)
        - u = (s · h) / a  (primera coordenada baricéntrica)
        - q = s × edge1  (usado para calcular v y t)
        - v = (D · q) / a  (segunda coordenada baricéntrica)
        - t = (edge2 · q) / a  (parámetro del rayo)

        Condiciones para intersección válida:
        1. a ≉ 0  (no paralelo)
        2. u ≥ 0  (dentro del triángulo)
        3. v ≥ 0  (dentro del triángulo)
        4. u + v ≤ 1  (dentro del triángulo)
        5. t_min < t < t_max  (en el rango válido)

        Interpretación de u, v:
        - u=0, v=0 → punto en v0
        - u=1, v=0 → punto en v1
        - u=0, v=1 → punto en v2
        - u+v=1 → punto en el borde v1-v2

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: Información del impacto si hay intersección válida

        Complejidad:
            O(1) - número fijo de operaciones independiente del tamaño de entrada

        Referencias:
            Möller, T., & Trumbore, B. (1997). Fast, minimum storage ray-triangle intersection.
            Journal of Graphics Tools, 2(1), 21-28.
        """
        # Algoritmo Möller-Trumbore
        edge1 = self.v1 - self.v0
        edge2 = self.v2 - self.v0
        h = ray.direction.cross(edge2)
        a = edge1.dot(h)

        # Si a es cercano a 0, el rayo es paralelo al triángulo
        if -1e-8 < a < 1e-8:
            return None

        f = 1.0 / a
        s = ray.origin - self.v0
        u = f * s.dot(h)

        if u < 0.0 or u > 1.0:
            return None

        q = s.cross(edge1)
        v = f * ray.direction.dot(q)

        if v < 0.0 or u + v > 1.0:
            return None

        # Calculamos t para ver dónde está la intersección en la línea del rayo
        t = f * edge2.dot(q)

        if t_min < t < t_max:
            intersection_point = ray.point_at(t)
            return HitRecord(
                t,
                intersection_point,
                self.normal,
                self.color,
                self.emission,
                self.is_metal,
                self.fuzz,
                self.is_dielectric,
                self.ior,
                obj_ref=self,
            )

        return None
