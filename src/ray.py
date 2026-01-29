from src.vector import Vec3


class Ray:
    """
    Representa un rayo en el espacio 3D.

    Un rayo es una **semirecta** (línea con un punto de inicio pero sin fin) que se usa
    para trazar la trayectoria de la luz en ray tracing. Es la estructura fundamental
    que permite simular cómo viaja la luz por la escena.

    Ecuación Paramétrica del Rayo:
        P(t) = O + t*D

        donde:
        - P(t) = punto en el rayo a distancia t
        - O = origen del rayo (punto de inicio)
        - D = dirección del rayo (vector unitario normalizado)
        - t = parámetro del rayo (distancia a lo largo de la dirección)
          * t = 0 → punto en el origen
          * t > 0 → puntos adelante del origen
          * t < 0 → puntos atrás del origen (generalmente no usados)

    Propiedades Importantes:
    1. **Dirección Normalizada**: La dirección SIEMPRE se normaliza (magnitud = 1)
       - Esto garantiza que t representa distancia real en unidades de escena
       - Simplifica cálculos de iluminación y física
       - Evita errores de escala en intersecciones

    2. **Origen Fijo**: El rayo siempre parte de un punto específico
       - Cámara: rayos parten del ojo/sensor
       - Rebotes: rayos parten del punto de impacto previo
       - Sombras: rayos parten de superficie hacia luz

    Usos en Ray Tracing:
    - **Rayos primarios**: Cámara → píxeles (visión)
    - **Rayos de sombra**: Superficie → luces (iluminación directa)
    - **Rayos de reflexión**: Superficie → dirección reflejada (espejos, metales)
    - **Rayos de refracción**: Superficie → dirección refractada (vidrio, agua)
    - **Rayos difusos**: Superficie → hemisferio aleatorio (materiales mate)

    Atributos:
        origin (Vec3): Punto de inicio del rayo en coordenadas 3D
        direction (Vec3): Vector dirección normalizado (magnitud = 1)

    Ejemplo:
        >>> # Rayo desde el origen hacia la derecha
        >>> ray = Ray(Vec3(0, 0, 0), Vec3(1, 0, 0))
        >>>
        >>> # Punto a distancia 5
        >>> point = ray.point_at(5)
        >>> print(point)  # Vec3(5, 0, 0)
        >>>
        >>> # Rayo desde cámara a través de un píxel
        >>> camera_pos = Vec3(0, 0, -10)
        >>> pixel_dir = Vec3(0.5, 0.3, 1.0).normalize()
        >>> camera_ray = Ray(camera_pos, pixel_dir)
    """

    def __init__(self, origin: Vec3, direction: Vec3):
        """
        Inicializa un rayo con un origen y una dirección.

        La dirección se normaliza automáticamente para garantizar que sea un
        vector unitario (magnitud = 1). Esto es CRÍTICO para que el parámetro t
        represente distancias reales.

        ¿Por qué normalizar?

        Sin normalización:
            direction = Vec3(10, 0, 0)  # Magnitud = 10
            t = 1 → P = origin + 1*Vec3(10,0,0) = origin + 10 unidades en X
            ❌ t NO representa distancia real (dice 1 pero avanza 10)

        Con normalización:
            direction = Vec3(10, 0, 0).normalize() = Vec3(1, 0, 0)  # Magnitud = 1
            t = 1 → P = origin + 1*Vec3(1,0,0) = origin + 1 unidad en X
            ✓ t SÍ representa distancia real (dice 1 y avanza 1)

        Esto afecta:
        - Cálculos de atenuación de luz (1/d²)
        - Comparaciones de distancia (objeto más cercano)
        - Física de materiales (probabilidades de scatter)
        - Debugging (valores de t interpretables)

        Args:
            origin (Vec3): Punto de inicio del rayo
            direction (Vec3): Vector dirección (será normalizado automáticamente)

        Nota:
            Si direction es el vector cero Vec3(0,0,0), normalize() puede causar
            división por cero. Asegúrate de pasar direcciones válidas.

        Ejemplo:
            >>> # Dirección ya normalizada
            >>> ray1 = Ray(Vec3(0,0,0), Vec3(1,0,0))  # direction.length() = 1
            >>>
            >>> # Dirección NO normalizada (se normalizará automáticamente)
            >>> ray2 = Ray(Vec3(0,0,0), Vec3(3,4,0))  # length = 5
            >>> print(ray2.direction)  # Vec3(0.6, 0.8, 0) con length = 1 ✓
        """
        self.origin = origin
        # Siempre normalizamos la dirección para evitar errores de escala
        self.direction = direction.normalize()

    def point_at(self, t: float) -> Vec3:
        """
        Calcula el punto P(t) en la recta del rayo usando la ecuación paramétrica.

        Implementa la ecuación fundamental:
            P(t) = O + t*D

        donde:
        - O = self.origin (punto de inicio)
        - D = self.direction (vector unitario)
        - t = parámetro de distancia

        Interpretación Geométrica:
        - Parte del origen O
        - Avanza t unidades en la dirección D
        - Retorna el punto final alcanzado

        Valores de t:
        - t = 0: Retorna el origen exacto
        - t > 0: Puntos adelante del origen (dirección de la flecha)
        - t < 0: Puntos atrás del origen (generalmente no usados en ray tracing)

        Args:
            t (float): Distancia a lo largo del rayo desde el origen
                      En unidades de la escena (ej: metros, unidades arbitrarias)

        Returns:
            Vec3: Las coordenadas 3D del punto en el rayo a distancia t

        Complejidad:
            O(1) - 3 multiplicaciones + 3 sumas (operación muy rápida)

        Ejemplos:
            >>> ray = Ray(Vec3(1, 2, 3), Vec3(1, 0, 0))
            >>>
            >>> # En el origen
            >>> ray.point_at(0)
            Vec3(1, 2, 3)  # Mismo que origin
            >>>
            >>> # 5 unidades adelante
            >>> ray.point_at(5)
            Vec3(6, 2, 3)  # Se movió 5 en X
            >>>
            >>> # 2.5 unidades adelante
            >>> ray.point_at(2.5)
            Vec3(3.5, 2, 3)
            >>>
            >>> # Atrás (raro, pero válido)
            >>> ray.point_at(-2)
            Vec3(-1, 2, 3)

        Uso en Ray Tracing:
            >>> # Encontramos intersección con esfera en t=7.3
            >>> hit_point = ray.point_at(7.3)
            >>>
            >>> # Calculamos normal, color, etc. en hit_point
            >>> normal = (hit_point - sphere.center).normalize()

        Relación con Intersecciones:
            Cuando un algoritmo de intersección (ej: rayo-esfera) retorna t,
            usamos point_at(t) para obtener las coordenadas exactas 3D del
            punto de impacto.
        """
        return self.origin + self.direction * t
