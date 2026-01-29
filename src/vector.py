import numpy as np


class Vec3:
    """
    Representa un vector tridimensional en el espacio euclidiano 3D.

    Vec3 es la **estructura de datos fundamental** en ray tracing. Se usa para
    representar prácticamente TODO en la escena:

    Usos Principales:
    - **Posiciones**: Coordenadas de puntos en el espacio (x, y, z)
    - **Direcciones**: Vectores unitarios (rayos, normales)
    - **Colores**: RGB donde (r, g, b) ∈ [0, 1] típicamente
    - **Velocidades**: Cambio de posición en el tiempo
    - **Fuerzas**: Física de partículas/simulaciones

    Representación Interna:
        Usa numpy array de 3 elementos float64 para:
        - ✅ Mayor precisión numérica (64 bits vs 32)
        - ✅ Operaciones SIMD optimizadas (vectorización)
        - ✅ Interoperabilidad con scipy/numpy

    Sistema de Coordenadas:
        Mano derecha (estándar en gráficos):

            Y (arriba)
            |
            |_____ X (derecha)
           /
          Z (hacia ti)

        - X positivo: Derecha
        - Y positivo: Arriba
        - Z positivo: Hacia delante (fuera de la pantalla)

    Inmutabilidad:
        Las operaciones NO modifican el vector original,
        siempre retornan un NUEVO Vec3:

        >>> v1 = Vec3(1, 2, 3)
        >>> v2 = v1 + Vec3(1, 1, 1)
        >>> print(v1)  # Vec3(1, 2, 3)  ← Sin cambios ✓
        >>> print(v2)  # Vec3(2, 3, 4)  ← Nuevo vector

    Atributos:
        components (np.ndarray): Array interno [x, y, z] en float64
        x (float): Componente X (property de solo lectura)
        y (float): Componente Y (property de solo lectura)
        z (float): Componente Z (property de solo lectura)

    Ejemplo de Uso:
        >>> # Posición de un punto
        >>> position = Vec3(5.0, 3.0, -2.0)
        >>>
        >>> # Dirección normalizada
        >>> direction = Vec3(1, 1, 0).normalize()
        >>> print(direction)  # Vec3(0.707..., 0.707..., 0)
        >>>
        >>> # Color rojo brillante
        >>> red_color = Vec3(1.0, 0.0, 0.0)
        >>>
        >>> # Operaciones
        >>> v1 = Vec3(1, 2, 3)
        >>> v2 = Vec3(4, 5, 6)
        >>> sum_vec = v1 + v2        # Vec3(5, 7, 9)
        >>> scaled = v1 * 2          # Vec3(2, 4, 6)
        >>> dot_result = v1.dot(v2)  # 32 (escalar)

    Nota sobre Precisión:
        Se usa float64 (no float32) porque:
        - Acumulación de errores en miles de rebotes
        - Intersecciones requieren alta precisión
        - Shadow acne puede empeorar con float32
        - Costo de memoria adicional es mínimo (~24 bytes vs 12)
    """

    def __init__(self, x, y, z):
        """
        Inicializa un vector 3D con componentes x, y, z.

        Los valores se convierten automáticamente a float64 para garantizar
        precisión numérica en cálculos subsecuentes.

        Args:
            x (float): Componente en el eje X
            y (float): Componente en el eje Y
            z (float): Componente en el eje Z

        Conversión Automática:
            >>> Vec3(1, 2, 3)      # Ints → float64 ✓
            >>> Vec3(1.0, 2.0, 3.0)  # Ya floats ✓
            >>> Vec3(1.5, 2.7, 3.2)  # Floats arbitrarios ✓

        Ejemplo:
            >>> # Vector posición
            >>> pos = Vec3(10.5, 20.3, -5.7)
            >>> print(pos.components)  # [10.5 20.3 -5.7]
            >>> print(pos.components.dtype)  # float64
            >>>
            >>> # Vector dirección
            >>> dir = Vec3(1, 0, 0)  # Dirección +X
            >>>
            >>> # Color verde
            >>> green = Vec3(0.0, 1.0, 0.0)

        Complejidad:
            O(1) - Creación de array numpy de tamaño fijo
        """
        # Usamos float64 para mayor precisión numérica
        self.components = np.array([x, y, z], dtype=np.float64)

    @property
    def x(self):
        """
        Componente X del vector (solo lectura).

        Returns:
            float: Valor de la coordenada X

        Ejemplo:
            >>> v = Vec3(5, 10, 15)
            >>> print(v.x)  # 5.0
        """
        return self.components[0]

    @property
    def y(self):
        """
        Componente Y del vector (solo lectura).

        Returns:
            float: Valor de la coordenada Y

        Ejemplo:
            >>> v = Vec3(5, 10, 15)
            >>> print(v.y)  # 10.0
        """
        return self.components[1]

    @property
    def z(self):
        """
        Componente Z del vector (solo lectura).

        Returns:
            float: Valor de la coordenada Z

        Ejemplo:
            >>> v = Vec3(5, 10, 15)
            >>> print(v.z)  # 15.0
        """
        return self.components[2]

    # --- Operaciones Aritméticas Básicas ---

    def __add__(self, other):
        """
        Suma de vectores (componente a componente).

        Implementa: v1 + v2 = (v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

        Interpretación Geométrica:
            Coloca el inicio de v2 en el final de v1.
            El resultado va desde el inicio de v1 hasta el final de v2.

        Visualización 2D (X-Y):
            v1 = (2, 1)    v2 = (1, 2)

               v2
              ↗
             •───→ v1+v2 = (3,3)
            ↗
           • v1
          O

        Args:
            other (Vec3): Vector a sumar

        Returns:
            Vec3: Nuevo vector resultante de la suma

        Propiedades Matemáticas:
            - Conmutativa: v1 + v2 = v2 + v1
            - Asociativa: (v1 + v2) + v3 = v1 + (v2 + v3)
            - Elemento identidad: v + Vec3(0,0,0) = v

        Usos en Ray Tracing:
            - Trasladar puntos: new_pos = pos + offset
            - Combinar direcciones: total = dir1 + dir2
            - Mezclar colores: mixed = color1 + color2

        Ejemplo:
            >>> v1 = Vec3(1, 2, 3)
            >>> v2 = Vec3(4, 5, 6)
            >>> result = v1 + v2
            >>> print(result)  # Vec3(5.0, 7.0, 9.0)
            >>>
            >>> # Trasladar esfera
            >>> sphere_center = Vec3(0, 0, 0)
            >>> offset = Vec3(5, 0, 0)
            >>> new_center = sphere_center + offset
            >>> print(new_center)  # Vec3(5.0, 0.0, 0.0)

        Complejidad:
            O(1) - 3 sumas
        """
        return Vec3(*(self.components + other.components))

    def __sub__(self, other):
        """
        Resta de vectores (componente a componente).

        Implementa: v1 - v2 = (v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)

        Interpretación Geométrica:
            Vector que va DESDE el final de v2 HASTA el final de v1.
            Equivale a: v1 + (-v2)

        Visualización:
            v1 = (3, 2)    v2 = (1, 1)

            v1 •
              ↗ \ v1-v2 = (2,1)
             /   ↘
            • v2  •
           O

        Args:
            other (Vec3): Vector a restar

        Returns:
            Vec3: Nuevo vector resultante de la resta

        Propiedades Matemáticas:
            - NO conmutativa: v1 - v2 ≠ v2 - v1
            - Anticonmutativa: v1 - v2 = -(v2 - v1)
            - v - v = Vec3(0, 0, 0)

        Usos en Ray Tracing:
            - Calcular dirección: dir = target - origin
            - Distancia relativa: offset = point2 - point1
            - Diferencia de colores: diff = color1 - color2

        Ejemplo:
            >>> v1 = Vec3(5, 7, 9)
            >>> v2 = Vec3(1, 2, 3)
            >>> result = v1 - v2
            >>> print(result)  # Vec3(4.0, 5.0, 6.0)
            >>>
            >>> # Dirección desde punto A a B
            >>> pointA = Vec3(0, 0, 0)
            >>> pointB = Vec3(10, 5, 3)
            >>> direction = pointB - pointA
            >>> print(direction)  # Vec3(10.0, 5.0, 3.0)
            >>>
            >>> # Normalizar para rayo
            >>> ray_dir = direction.normalize()

        Complejidad:
            O(1) - 3 restas
        """
        return Vec3(*(self.components - other.components))

    def __mul__(self, other):
        """
        Multiplicación por escalar O multiplicación componente a componente.

        DOS MODOS de operación:

        1. **Multiplicación por Escalar** (other = número):
           v * k = (v.x*k, v.y*k, v.z*k)

           Escala el vector (cambia magnitud, mantiene dirección)

        2. **Multiplicación Componente a Componente** (other = Vec3):
           v1 * v2 = (v1.x*v2.x, v1.y*v2.y, v1.z*v2.z)

           También llamada multiplicación Hadamard o element-wise

        Args:
            other (float | Vec3): Escalar o vector para multiplicar

        Returns:
            Vec3: Nuevo vector resultante

        Modo 1 - Escalar:
            Interpretación Geométrica:
            - k > 1: Vector más largo (misma dirección)
            - k = 1: Sin cambio
            - 0 < k < 1: Vector más corto (misma dirección)
            - k = 0: Vector cero
            - k < 0: Vector invertido y escalado

            Ejemplo:
                >>> v = Vec3(1, 2, 3)
                >>> v2 = v * 2      # Vec3(2, 4, 6)  ← Doble largo
                >>> v_half = v * 0.5  # Vec3(0.5, 1, 1.5)  ← Mitad
                >>> v_neg = v * -1  # Vec3(-1, -2, -3)  ← Invertido

        Modo 2 - Componente a Componente:
            Usos:
            - **Colores**: Modular RGB (atenuación)
              >>> light = Vec3(1.0, 1.0, 1.0)   # Luz blanca
              >>> surface = Vec3(0.8, 0.2, 0.2)  # Superficie roja
              >>> reflected = light * surface
              >>> print(reflected)  # Vec3(0.8, 0.2, 0.2)  ← Luz roja

            - **Máscaras**: Aplicar por componente
              >>> vec = Vec3(5, 10, 15)
              >>> mask = Vec3(1, 0, 1)
              >>> result = vec * mask
              >>> print(result)  # Vec3(5, 0, 15)  ← Y eliminado

        NOTA: Esto NO es producto punto (dot) ni producto cruz (cross).

        Complejidad:
            O(1) - 3 multiplicaciones
        """
        # Multiplicación por escalar o por otro vector (elemento a elemento)
        if isinstance(other, Vec3):
            return Vec3(*(self.components * other.components))
        return Vec3(*(self.components * other))

    def __truediv__(self, scalar):
        """
        División por escalar (componente a componente).

        Implementa: v / k = (v.x/k, v.y/k, v.z/k)

        Equivale a multiplicar por 1/k escalando el vector hacia abajo.

        Args:
            scalar (float): Escalar divisor (debe ser ≠ 0)

        Returns:
            Vec3: Nuevo vector resultante

        Precaución:
            Si scalar = 0 → División por cero ❌
            NumPy retornará inf, -inf o nan

        Interpretación Geométrica:
            - scalar > 1: Vector más corto
            - scalar = 1: Sin cambio
            - 0 < scalar < 1: Vector más largo
            - scalar < 0: Invertido y escalado

        Usos en Ray Tracing:
            - Normalización: v / v.length()
            - Promedio: (v1 + v2 + v3) / 3
            - Interpolación: v1 + (v2 - v1) / 2

        Ejemplo:
            >>> v = Vec3(10, 20, 30)
            >>> v_half = v / 2
            >>> print(v_half)  # Vec3(5.0, 10.0, 15.0)
            >>>
            >>> # Normalización manual
            >>> v = Vec3(3, 4, 0)
            >>> length = v.length()  # 5.0
            >>> normalized = v / length
            >>> print(normalized)  # Vec3(0.6, 0.8, 0.0)
            >>> print(normalized.length())  # 1.0 ✓

        Complejidad:
            O(1) - 3 divisiones
        """
        return Vec3(*(self.components / scalar))

    # --- Operaciones de Álgebra Lineal Clave ---

    def dot(self, other):
        """
        Producto punto (dot product, producto escalar, producto interno).

        Fórmula:
            v1 · v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
                    = |v1| |v2| cos(θ)

        donde θ = ángulo entre los vectores.

        Returns:
            float: Escalar resultante (NO un vector)

        Interpretación Geométrica:
            - Proyección de v1 sobre v2 (o viceversa)
            - Mide cuán "alineados" están los vectores

        Propiedades:
            - Conmutativo: v1 · v2 = v2 · v1
            - Distributivo: v1 · (v2 + v3) = v1·v2 + v1·v3
            - v · v = |v|² (magnitud al cuadrado)

        Casos Especiales:
            v1 · v2 > 0  → θ < 90°  (ángulo agudo, misma dirección general)
            v1 · v2 = 0  → θ = 90°  (perpendiculares, ortogonales)
            v1 · v2 < 0  → θ > 90°  (ángulo obtuso, direcciones opuestas)

        Para Vectores Unitarios (|v1| = |v2| = 1):
            v1 · v2 = cos(θ)

            Ejemplos:
            - Paralelos (θ=0°): dot = 1
            - Perpendiculares (θ=90°): dot = 0
            - Opuestos (θ=180°): dot = -1

        Usos en Ray Tracing:
            1. **Iluminación** (Ley de Lambert):
               brightness = max(0, normal · light_dir)

            2. **Test de hemisferio**:
               if ray_dir · normal > 0: mismo_lado()

            3. **Cálculo de ángulos**:
               theta = arccos(v1 · v2)  # Si unitarios

            4. **Proyección**:
               proj = (v1 · v2) / |v2|²  * v2

            5. **Reflexión**:
               reflected = incident - 2*(incident · normal)*normal

        Ejemplo Numérico:
            >>> v1 = Vec3(1, 0, 0)  # Dirección +X
            >>> v2 = Vec3(1, 0, 0)  # También +X
            >>> print(v1.dot(v2))  # 1.0 (paralelos)
            >>>
            >>> v3 = Vec3(0, 1, 0)  # Dirección +Y
            >>> print(v1.dot(v3))  # 0.0 (perpendiculares)
            >>>
            >>> v4 = Vec3(-1, 0, 0)  # Dirección -X
            >>> print(v1.dot(v4))  # -1.0 (opuestos)
            >>>
            >>> # Iluminación difusa
            >>> normal = Vec3(0, 1, 0)  # Superficie horizontal arriba
            >>> light = Vec3(0.6, 0.8, 0).normalize()
            >>> brightness = max(0, normal.dot(light))
            >>> print(brightness)  # 0.8 (80% iluminado)

        Complejidad:
            O(1) - 3 multiplicaciones + 2 sumas
        """
        return np.dot(self.components, other.components)

    def cross(self, other):
        """
        Producto cruz (cross product, producto vectorial).

        Fórmula:
            v1 × v2 = (v1.y*v2.z - v1.z*v2.y,
                      v1.z*v2.x - v1.x*v2.z,
                      v1.x*v2.y - v1.y*v2.x)

        Returns:
            Vec3: Vector perpendicular a ambos v1 y v2

        Propiedades Críticas:
            1. **Perpendicular**: (v1 × v2) · v1 = 0  Y  (v1 × v2) · v2 = 0
            2. **NO conmutativo**: v1 × v2 = -(v2 × v1)
            3. **Regla mano derecha**: Dirección determinada por orientación
            4. **Magnitud**: |v1 × v2| = |v1| |v2| sin(θ)

        Interpretación Geométrica:
            - Resultado apunta perpendicular al plano formado por v1 y v2
            - Magnitud = área del paralelogramo formado por v1 y v2
            - Dirección según regla mano derecha

        Regla Mano Derecha:
            Dedos: v1 → Palma: v2 → Pulgar: v1 × v2

                ↑ v1×v2 (pulgar)
                |
                •----→ v2 (palma)
               /
              v1 (dedos)

        Casos Especiales:
            - v1 y v2 paralelos → v1 × v2 = Vec3(0,0,0)
            - v1 y v2 perpendiculares → |resultado| = máximo
            - v × v = Vec3(0,0,0)

        Usos en Ray Tracing:
            1. **Calcular normales** de triángulos:
               normal = (v2 - v1) × (v3 - v1)

            2. **Sistema de coordenadas**:
               up = Vec3(0, 1, 0)
               right = forward × up
               actual_up = right × forward

            3. **Test de orientación** (punto dentro/fuera polígono)

            4. **Cálculo de tangentes** para texturas

        Ejemplo Numérico:
            >>> # Ejes canónicos
            >>> x_axis = Vec3(1, 0, 0)
            >>> y_axis = Vec3(0, 1, 0)
            >>> z_axis = Vec3(0, 0, 1)
            >>>
            >>> # X × Y = Z (regla mano derecha)
            >>> result = x_axis.cross(y_axis)
            >>> print(result)  # Vec3(0.0, 0.0, 1.0) ✓
            >>>
            >>> # Y × X = -Z (anticonmutativo)
            >>> result2 = y_axis.cross(x_axis)
            >>> print(result2)  # Vec3(0.0, 0.0, -1.0) ✓
            >>>
            >>> # Normal de triángulo
            >>> v1 = Vec3(1, 0, 0)
            >>> v2 = Vec3(0, 1, 0)
            >>> v3 = Vec3(0, 0, 1)
            >>> edge1 = v2 - v1
            >>> edge2 = v3 - v1
            >>> normal = edge1.cross(edge2).normalize()
            >>> print(normal)  # Perpendicular al plano

        Complejidad:
            O(1) - 6 multiplicaciones + 3 restas
        """
        return Vec3(*np.cross(self.components, other.components))

    def length(self):
        """
        Magnitud (norma, longitud euclidiana) del vector.

        Fórmula:
            |v| = √(x² + y² + z²)

        Returns:
            float: Magnitud del vector (siempre ≥ 0)

        Interpretación Geométrica:
            Distancia desde el origen (0,0,0) hasta el punto (x,y,z).
            Teorema de Pitágoras en 3D.

        Visualización 2D:
            v = (3, 4)

              (3,4)
             /|
            / | 4
          5/  |
          /   |
         O----+
            3

        |v| = √(3² + 4²) = √(9 + 16) = √25 = 5

        Propiedades:
            - |v| = 0  ⟺  v = Vec3(0,0,0)
            - |k*v| = |k| * |v|  (escalar absoluto)
            - |v1 + v2| ≤ |v1| + |v2|  (desigualdad triangular)

        Usos en Ray Tracing:
            - **Distancias**: d = (p2 - p1).length()
            - **Normalización**: v_unit = v / v.length()
            - **Atenuación de luz**: intensity / distance²
            - **Verificaciones**: if v.length() < epsilon: ...

        Ejemplo:
            >>> v = Vec3(3, 4, 0)
            >>> print(v.length())  # 5.0 ✓
            >>>
            >>> # Distancia entre puntos
            >>> p1 = Vec3(0, 0, 0)
            >>> p2 = Vec3(1, 1, 1)
            >>> distance = (p2 - p1).length()
            >>> print(distance)  # √3 ≈ 1.732
            >>>
            >>> # Vector ya normalizado
            >>> unit = Vec3(1, 0, 0)
            >>> print(unit.length())  # 1.0 ✓

        Complejidad:
            O(1) - 3 multiplicaciones + 2 sumas + 1 sqrt

        Optimización:
            Si solo necesitas COMPARAR distancias (no el valor exacto):
            Usa length_squared() = x² + y² + z² (sin sqrt, más rápido)

            Ejemplo:
            if (p - center).length() < radius:  # Lento (sqrt)
            if (p - center).dot(p - center) < radius²:  # Rápido ✓
        """
        return np.linalg.norm(self.components)

    def normalize(self):
        """
        Convierte el vector en unitario (magnitud = 1), manteniendo su dirección.

        Fórmula:
            v̂ = v / |v|

        Returns:
            Vec3: Nuevo vector con magnitud 1 (o vector cero si |v| = 0)

        Propiedades del Vector Normalizado:
            - Magnitud: |v̂| = 1
            - Dirección: Misma que v
            - Dir contraria: -v̂

        Manejo de Vector Cero:
            Si |v| = 0 (división por cero), retorna Vec3(0,0,0)
            Esto evita excepciones pero debes verificar externamente si es crítico.

        ¿Por Qué Normalizar?

        1. **Direcciones de rayos**:
           Para que t represente distancia real en Ray.point_at(t)

        2. **Normales de superficie**:
           Los cálculos de iluminación requieren vectores unitarios

        3. **Cálculos de ángulos**:
           Si v1 y v2 son unitarios: v1 · v2 = cos(θ)

        4. **Consistencia numérica**:
           Evita escalas arbitrarias en algoritmos

        Usos en Ray Tracing:
            - **Rayos**: Ray(origin, direction.normalize())
            - **Normales**: normal = (p - center).normalize()
            - **Reflexión**: reflected = incident.reflect(normal.normalize())
            - **Cámaras**: look_dir.normalize()

        Ejemplo:
            >>> v = Vec3(3, 4, 0)
            >>> print(v.length())  # 5.0
            >>>
            >>> v_norm = v.normalize()
            >>> print(v_norm)  # Vec3(0.6, 0.8, 0.0)
            >>> print(v_norm.length())  # 1.0 ✓
            >>>
            >>> # Dirección desde A a B
            >>> A = Vec3(0, 0, 0)
            >>> B = Vec3(10, 10, 10)
            >>> direction = (B - A).normalize()
            >>> print(direction)  # Vec3(0.577..., 0.577..., 0.577...)
            >>> print(direction.length())  # 1.0 ✓
            >>>
            >>> # Vector cero (caso especial)
            >>> zero = Vec3(0, 0, 0)
            >>> zero_norm = zero.normalize()
            >>> print(zero_norm)  # Vec3(0.0, 0.0, 0.0)

        Complejidad:
            O(1) - 1 length() + 3 divisiones

        IMPORTANTE:
            El vector original NO se modifica (inmutabilidad):

            >>> v = Vec3(3, 4, 0)
            >>> v_norm = v.normalize()
            >>> print(v.length())  # 5.0  ← SIN CAMBIO
            >>> print(v_norm.length())  # 1.0  ← NORMALIZADO
        """
        mag = self.length()
        if mag == 0:
            return Vec3(0, 0, 0)
        return self / mag

    def reflect(self, normal):
        """
        Calcula el vector de reflexión especular según una normal.

        Fórmula (reflexión perfecta):
            R = I - 2(I · N)N

        donde:
        - I = vector incidente (self)
        - N = normal de la superficie (debe estar normalizada)
        - R = vector reflejado

        Args:
            normal (Vec3): Normal de la superficie (DEBE ser unitaria |N|=1)

        Returns:
            Vec3: Vector reflejado

        Derivación Matemática:

            N (normal)
            ↑
            |     R (reflejado)
            |    ↗
            •---------  Superficie
            |  ↘
            |   I (incidente)

        Proyección de I sobre N:
            proj = (I · N) * N

        Componente perpendicular:
            perp = I - proj

        Reflexión:
            R = -perp - proj
              = -(I - proj) - proj
              = -I + proj - proj
              = -I + 2proj
              = I - 2(I · N)N  ✓

        Casos Especiales:
            - I perpendicular a N (I · N = 0): R = I (sin reflexión)
            - I paralelo a N (I · N = ±1): R = -I (rebote directo)

        CRÍTICO:
            La normal DEBE estar normalizada (|N| = 1).
            Si no lo está, el resultado será INCORRECTO.

            >>> normal = some_vector.normalize()  # ✓ CORRECTO
            >>> reflected = incident.reflect(normal)

        Usos en Ray Tracing:
            1. **Materiales especulares** (espejos):
               reflected_ray = Ray(hit_point, incident.reflect(normal))

            2. **Metales** con reflexión perfecta o fuzzy

            3. **Reflexión parcial** en dieléctricos (Fresnel)

        Ejemplo Numérico:
            >>> # Rayo vertical golpea superficie horizontal
            >>> incident = Vec3(0, -1, 0)  # Hacia abajo
            >>> normal = Vec3(0, 1, 0)     # Apunta arriba
            >>>
            >>> reflected = incident.reflect(normal)
            >>> print(reflected)  # Vec3(0.0, 1.0, 0.0)  ← Hacia arriba ✓
            >>>
            >>> # Rayo diagonal
            >>> incident = Vec3(1, -1, 0).normalize()
            >>> normal = Vec3(0, 1, 0)
            >>> reflected = incident.reflect(normal)
            >>> print(reflected)  # Vec3(0.707..., 0.707..., 0)
            >>> # Ángulo de salida = ángulo de entrada ✓
            >>>
            >>> # Verificar: R · N debe ser opuesto a I · N
            >>> print(incident.dot(normal))   # -0.707 (entrante)
            >>> print(reflected.dot(normal))  # 0.707 (saliente) ✓

        Complejidad:
            O(1) - 1 dot + 3 multiplicaciones + 3 restas

        Física:
            Esta es la "Ley de Reflexión": ángulo incidencia = ángulo reflexión
            Medidos desde la normal.
        """
        return self - normal * (2 * self.dot(normal))

    def __repr__(self):
        """
        Representación en string del vector para debugging.

        Returns:
            str: Formato legible "Vec3(x, y, z)"

        Ejemplo:
            >>> v = Vec3(1.5, 2.7, 3.2)
            >>> print(v)  # Vec3(1.5, 2.7, 3.2)
            >>>
            >>> # En listas
            >>> vecs = [Vec3(1,2,3), Vec3(4,5,6)]
            >>> print(vecs)  # [Vec3(1.0, 2.0, 3.0), Vec3(4.0, 5.0, 6.0)]
        """
        return f"Vec3({self.x}, {self.y}, {self.z})"
