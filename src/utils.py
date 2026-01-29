import random
from src.vector import Vec3
from src.geometry import Triangle


def random_in_unit_sphere():
    """
    Genera un vector aleatorio UNIFORMEMENTE distribuido dentro de una esfera de radio 1.

    Este es uno de los métodos fundamentales en Monte Carlo ray tracing para
    generar direcciones aleatorias en materiales difusos (Lambertian).

    Algoritmo de Rechazo (Rejection Sampling):
    1. Genera punto aleatorio en cubo [-1, 1]³
    2. Verifica si está dentro de la esfera (distancia < 1)
    3. Si SÍ → retorna el punto ✓
    4. Si NO → rechaza y vuelve al paso 1

    ¿Por qué método de rechazo?

    INCORRECTO - Coordenadas esféricas ingenuas:
        theta = random() * 2π
        phi = random() * π
        r = random()

        → Genera MÁS puntos cerca del centro (distribución NO uniforme)

    CORRECTO - Método de rechazo:
        → Distribución perfectamente uniforme en TODO el volumen
        → Simple de implementar
        → Eficiencia ~52% (acepta π/6 ≈ 52% de puntos)

    Probabilidad Matemática:
        P(aceptar) = Volumen_esfera / Volumen_cubo
                   = (4/3)π / 8
                   = π/6
                   ≈ 0.524 (52.4%)

    Iteraciones Esperadas:
        E[iteraciones] = 1 / P(aceptar)
                       = 6/π
                       ≈ 1.91 veces

    Returns:
        Vec3: Vector aleatorio dentro de la esfera unitaria
              Magnitud: 0 ≤ |v| < 1
              Distribución: UNIFORME en volumen

    Complejidad:
        O(1) esperado (~2 iteraciones)
        Peor caso: O(∞) teórico (improbable)

    Usos:
        - Scattering difuso (Lambertian)
        - Generación de ray offsets
        - Depth of field (desenfoque)
        - Inicialización de partículas

    Ejemplo:
        >>> # 1000 puntos aleatorios en esfera
        >>> points = [random_in_unit_sphere() for _ in range(1000)]
        >>>
        >>> # Todos están dentro de la esfera
        >>> assert all(p.length() < 1.0 for p in points)
        >>>
        >>> # Usar para scatter difuso
        >>> scatter_dir = hit_point + random_in_unit_sphere()

    Alternativas:
        - random_unit_vector(): Punto en la SUPERFICIE (length = 1)
        - random_in_hemisphere(n): Punto en hemisferio orientado por n
    """
    while True:
        # Generamos un punto en un cubo de -1 a 1
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

        # ¿Está dentro de la esfera? (Método de rechazo)
        if p.length() < 1.0:
            return p


def generar_direccion_aleatoria(normal):
    """
    Genera una dirección unitaria aleatoria en el hemisferio orientado por la normal.

    Esta función es FUNDAMENTAL para materiales difusos (Lambertian) en path tracing.
    Implementa reflexión difusa: la luz rebota aleatoriamente en todas direcciones
    del hemisferio "visible" (el que apunta hacia afuera de la superficie).

    Algoritmo:
    1. Genera vector aleatorio uniforme en esfera completa
    2. Normaliza → punto en la SUPERFICIE de la esfera
    3. Test de hemisferio: ¿Está del mismo lado que la normal?
       - Usa producto punto: n · v > 0 significa ángulo < 90°
    4. Si está en hemisferio correcto → retorna
    5. Si está en hemisferio opuesto → invierte (multiplica por -1)

    Producto Punto para Test de Hemisferio:
        n · v = |n| |v| cos(θ)

        Como n y v son unitarios (|n| = |v| = 1):
        n · v = cos(θ)

        Si n · v > 0 → cos(θ) > 0 → θ < 90° → Mismo hemisferio ✓
        Si n · v < 0 → cos(θ) < 0 → θ > 90° → Hemisferio opuesto ✗
        Si n · v = 0 → cos(θ) = 0 → θ = 90° → En el plano tangente (borde)

    Visualización:

        Normal ↑         Hemisferio correcto (v · n > 0)
               |    •  •
               |  •      •
        -------+-------→------  Plano tangente (v · n = 0)
               |
               |  ✗ Hemisferio incorrecto (v · n < 0)

    Distribución:
        - Uniforme en ÁREA del hemisferio
        - NO es distribución coseno-ponderada (Lambertian ideal)
        - Para Lambertian exacto, usar técnicas más avanzadas

    Args:
        normal (Vec3): Normal de la superficie (debe estar normalizada, |n| = 1)

    Returns:
        Vec3: Vector unitario aleatorio en el hemisferio de la normal
              Magnitud: |v| = 1
              Ángulo con normal: 0° ≤ θ ≤ 90°

    Complejidad:
        O(1) esperado
        ~2 iteraciones para random_in_unit_sphere()

    Efecto en Path Tracing:
        Esta función determina cómo "rebota" la luz en materiales mate:
        - Luz incidente desde cualquier ángulo
        - Se dispersa aleatoriamente en el hemisferio
        - Múltiples rebotes crean iluminación global suave

    Ejemplo Numérico:
        >>> # Superficie horizontal apuntando arriba
        >>> normal = Vec3(0, 1, 0)
        >>>
        >>> # Generar 3 direcciones
        >>> dir1 = generar_direccion_aleatoria(normal)
        >>> # Ejemplo: Vec3(0.3, 0.8, 0.5)
        >>>
        >>> # Verificar que está en hemisferio superior
        >>> assert dir1.dot(normal) > 0  # ✓ Apunta "hacia arriba"
        >>> assert dir1.y > 0  # ✓ Componente Y positiva

    Uso en Path Tracing:
        >>> # Material difuso golpeado
        >>> hit_record = sphere.hit(ray, 0.001, inf)
        >>>
        >>> # Generar dirección de rebote
        >>> scatter_direction = generar_direccion_aleatoria(hit_record.normal)
        >>>
        >>> # Nuevo rayo rebotado
        >>> scattered_ray = Ray(hit_record.point, scatter_direction)
        >>>
        >>> # Continuar trazado recursivamente
        >>> color = trace_ray(scattered_ray, depth - 1)

    Comparación con Otras Distribuciones:

        Uniforme en hemisferio (esta función):
        - Todos los ángulos equiprobables
        - Simple y rápido
        - Aproximación razonable

        Coseno-ponderado (Lambertian ideal):
        - Más muestras cerca de la normal
        - Físicamente más preciso
        - Más complejo de implementar
    """
    # 1. Obtenemos un punto aleatorio normalizado (está en la superficie de la esfera)
    random_dir = random_in_unit_sphere().normalize()

    # 2. Verificamos si está en el mismo hemisferio que la normal
    # Si el producto punto es > 0, el ángulo es < 90° (está "hacia afuera")
    if random_dir.dot(normal) > 0.0:
        return random_dir
    else:
        # Si apunta hacia adentro, lo invertimos
        return random_dir * -1


def load_obj(filename, color, offset=Vec3(0, 0, 0), scale=1.0, material_params=None):
    """
    Carga un archivo .obj (Wavefront OBJ) y lo convierte en lista de triángulos.

    El formato OBJ es el estándar de la industria para modelos 3D. Permite
    importar geometría compleja (personajes, objetos, arquitectura) creada
    en software 3D (Blender, Maya, 3DS Max) al ray tracer.

    Formato OBJ Básico:

        # Comentario
        v x y z          # Vértice (coordenadas 3D)
        vt u v           # Coordenada de textura (ignorada)
        vn x y z         # Normal (ignorada, se recalcula)
        f v1 v2 v3       # Cara triangular (índices de vértices)
        f v1/vt1 v2/vt2 v3/vt3              # Con texturas
        f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3  # Con texturas y normales

    Ejemplo de archivo OBJ:
        ```
        # Cubo simple
        v -1 -1 -1
        v  1 -1 -1
        v  1  1 -1
        v -1  1 -1
        # ... más vértices

        f 1 2 3    # Triángulo con vértices 1, 2, 3
        f 1 3 4    # Otro triángulo
        ```

    Características de esta Implementación:

    1. **Vértices (v)**:
       - Lee coordenadas X, Y, Z
       - Aplica escala: v * scale
       - Aplica offset: v + offset

    2. **Caras (f)**:
       - Soporta f v1 v2 v3 (simple)
       - Soporta f v1/vt1/vn1 ... (con texturas/normales)
       - Índices en OBJ empiezan en 1 (se convierte a base-0)
       - Soporta polígonos n-lados (triangula con "fan")

    3. **Triangulación de Polígonos**:
       Si la cara tiene >3 vértices (cuadrado, pentágono, etc.):
       ```
       f 1 2 3 4  (cuadrado)

       Se convierte en:
       ▶ Triángulo 1: [1, 2, 3]
       ▶ Triángulo 2: [1, 3, 4]

       (Método "fan" desde primer vértice)
       ```

    4. **Material**:
       - Todos los triángulos comparten mismo color y propiedades
       - Usa material_params para metales/dieléctricos

    Parámetros de Transformación:

    **scale** - Escala uniforme del modelo:
        - scale = 1.0: Tamaño original
        - scale = 2.0: Doble de grande
        - scale = 0.5: Mitad de grande

        Útil cuando el modelo OBJ está en unidades diferentes
        (ej: centímetros vs metros)

    **offset** - Traslación del modelo:
        - Vec3(0, 0, 0): Posición original
        - Vec3(5, 0, 0): Desplazado 5 unidades en X
        - Vec3(0, 2, 0): Elevado 2 unidades en Y

        Útil para posicionar el modelo en la escena

    Args:
        filename (str): Ruta al archivo .obj
        color (Vec3): Color RGB [0-1] para todos los triángulos
        offset (Vec3, opcional): Desplazamiento del modelo. Por defecto (0,0,0)
        scale (float, opcional): Factor de escala. Por defecto 1.0
        material_params (dict, opcional): Propiedades del material:
            - 'is_metal': bool
            - 'fuzz': float [0-1]
            - 'is_dielectric': bool
            - 'ior': float
            Por defecto: material difuso simple

    Returns:
        list[Triangle]: Lista de triángulos que componen el modelo
                        Lista vacía si hay error de carga

    Manejo de Errores:
        - Archivo no encontrado → imprime error, retorna []
        - Formato inválido → imprime error, retorna []
        - La carga NO lanza excepciones (fail-safe)

    Complejidad:
        - Tiempo: O(V + F) donde V=vértices, F=caras
        - Memoria: O(V + T) donde T=triángulos generados

    Ejemplo de Uso:
        >>> # Cargar modelo de tetera
        >>> teapot_triangles = load_obj(
        ...     'models/teapot.obj',
        ...     color=Vec3(0.8, 0.1, 0.1),  # Rojo
        ...     offset=Vec3(0, 1, 0),       # Elevada 1 unidad
        ...     scale=2.0,                   # Doble de grande
        ...     material_params={'is_metal': True, 'fuzz': 0.1}
        ... )
        >>>
        >>> print(f"Cargados {len(teapot_triangles)} triángulos")
        >>> # Cargados 2464 triángulos
        >>>
        >>> # Agregar a la escena
        >>> scene.extend(teapot_triangles)

    Ejemplo con BVH:
        >>> # Cargar modelo complejo
        >>> dragon = load_obj('dragon.obj', Vec3(0.2, 0.8, 0.2))
        >>>
        >>> # Crear BVH para aceleración
        >>> dragon_bvh = BVHNode.create(dragon)
        >>>
        >>> # Agregar BVH a la escena
        >>> scene.append(dragon_bvh)
        >>> # Ahora el dragón se renderiza eficientemente

    Limitaciones:
        - NO soporta materiales por vértice/cara
        - NO soporta texturas (coordenadas vt ignoradas)
        - NO soporta normales suavizadas (vn ignoradas, se recalculan)
        - NO soporta grupos (g) ni objetos (o)

        Para estas características, se necesitaría parser OBJ más avanzado.

    Formato de Índices Negativos:
        OBJ permite índices negativos (relativos al final):
        ```
        v 0 0 0
        v 1 0 0
        v 0 1 0
        f -3 -2 -1   # Últimos 3 vértices
        ```

        Esta implementación los soporta:
        idx = idx - 1 if idx > 0 else len(vertices) + idx

    Archivos OBJ Típicos:
        - Simples: ~100 triángulos (cubos, esferas low-poly)
        - Medianos: ~1000 triángulos (personajes, muebles)
        - Complejos: ~100,000+ triángulos (escáneres 3D, modelos detallados)

    Rendimiento:
        Sin BVH: Cada triángulo se prueba individualmente (LENTO para >100 tris)
        Con BVH: Estructura de aceleración permite modelos de millones de tris

        SIEMPRE usar BVH para models cargados de OBJ.
    """
    vertices = []
    triangles = []

    # Parámetros de material por defecto
    m_params = (
        material_params
        if material_params
        else {"is_metal": False, "fuzz": 0.0, "is_dielectric": False, "ior": 1.5}
    )

    try:
        with open(filename, "r") as f:
            for line in f:
                if line.startswith("v "):
                    # Vértices: v x y z
                    parts = line.split()
                    v = Vec3(float(parts[1]), float(parts[2]), float(parts[3]))
                    vertices.append(v * scale + offset)

                elif line.startswith("f "):
                    # Caras: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                    # (Solo nos interesan los índices de los vértices)
                    parts = line.split()[1:]
                    indices = []
                    for p in parts:
                        # El primer número antes de la barra es el índice del vértice
                        idx = int(p.split("/")[0])
                        # Los archivos OBJ usan índices que empiezan en 1
                        indices.append(idx - 1 if idx > 0 else len(vertices) + idx)

                    # Creamos el triángulo con los vértices correspondientes
                    # Soporta polígonos de más de 3 lados haciendo un "fan"
                    for i in range(1, len(indices) - 1):
                        triangles.append(
                            Triangle(
                                vertices[indices[0]],
                                vertices[indices[i]],
                                vertices[indices[i + 1]],
                                color,
                                **m_params,
                            )
                        )
        print(f"Modelo cargado: {len(triangles)} triángulos.")
    except Exception as e:
        print(f"Error al cargar el archivo OBJ: {e}")

    return triangles


def random_in_unit_disk():
    """
    Genera un punto aleatorio uniformemente distribuido dentro de un disco de radio 1.

    Un disco es un círculo 2D (plano XY, Z=0). Esta función es fundamental para
    simular profundidad de campo (depth of field) en cámaras reales.

    Algoritmo de Rechazo 2D:
    1. Genera punto aleatorio en cuadrado [-1, 1]²
    2. Calcula distancia desde centro
    3. Si distancia < 1 → está dentro del disco ✓
    4. Si distancia ≥ 1 → rechaza, vuelve al paso 1

    Probabilidad Matemática:
        P(aceptar) = Área_círculo / Área_cuadrado
                   = π / 4
                   ≈ 0.785 (78.5%)

    Iteraciones Esperadas:
        E[iteraciones] = 1 / P(aceptar)
                       = 4/π
                       ≈ 1.27 veces

    Más eficiente que random_in_unit_sphere() (78.5% vs 52.4%)

    Uso Principal: Depth of Field (Profundidad de Campo)

    En cámaras reales:
    - La lente tiene apertura finita (no es punto)
    - Rayos desde diferentes puntos de la lente convergen en el plano focal
    - Objetos fuera del plano focal aparecen desenfocados

    Simulación:
        Para cada píxel:
        1. Calcula dirección del rayo al píxel
        2. Genera offset aleatorio en disco (aperture * random_in_unit_disk())
        3. Ajusta origen del rayo con este offset
        4. Mantén el punto focal fijo
        5. Resultado: Bokeh natural (círculos de confusión)

    Plano Focal:
        - Objetos EN el plano focal → nítidos
        - Objetos ANTES → desenfocados
        - Objetos DESPUÉS → desenfocados

        Efecto aumenta con distancia al plano focal.

    Returns:
        Vec3: Punto 2D aleatorio dentro del disco
              - x, y: Coordenadas en [-1, 1]
              - z: Siempre 0
              - Distancia desde centro: 0 ≤ r < 1
              - Distribución: UNIFORME en área

    Complejidad:
        O(1) esperado (~1.3 iteraciones)

    Visualización del Disco:

            y
            ↑
        -1  +-------+ 1
            |  ...  |     Puntos en disco (aceptados)
            | ..:.. |     Región entre círculo y cuadrado (rechazados)
         0  |.::o::.|  ← Centro (0, 0)
            | ..:.. |
            |  ...  |
        -1  +-------+ → x
           -1   0   1

    Ejemplo de Uso - Depth of Field:
        >>> # Configuración de cámara
        >>> aperture = 0.1  # Apertura (pequeña = más enfoque)
        >>> focus_distance = 10.0  # Distancia al plano focal
        >>>
        >>> # Para un píxel
        >>> pixel_direction = calculate_pixel_direction(px, py)
        >>>
        >>> # Offset aleatorio en lente
        >>> lens_offset = random_in_unit_disk() * aperture
        >>>
        >>> # Origen del rayo (mover en plano de lente)
        >>> ray_origin = camera_pos + lens_offset.x * camera_right + lens_offset.y * camera_up
        >>>
        >>> # Punto focal (donde deben converger los rayos)
        >>> focal_point = camera_pos + focus_distance * pixel_direction
        >>>
        >>> # Dirección ajustada
        >>> ray_direction = (focal_point - ray_origin).normalize()
        >>>
        >>> # Rayo con DOF
        >>> ray = Ray(ray_origin, ray_direction)

    Ejemplo Numérico:
        >>> # Generar 5 puntos
        >>> for _ in range(5):
        >>>     p = random_in_unit_disk()
        >>>     print(f"({p.x:.2f}, {p.y:.2f}), dist={p.length():.2f}")
        >>>
        >>> # Salida posible:
        >>> # (0.34, -0.67), dist=0.75
        >>> # (-0.12, 0.45), dist=0.47
        >>> # (0.89, 0.23), dist=0.92
        >>> # (-0.56, -0.78), dist=0.96
        >>> # (0.01, -0.34), dist=0.34
        >>>
        >>> # Todos cumplen: dist < 1.0 ✓

    Efecto de Apertura en DOF:

        aperture = 0.0:
        - Sin offset (lente puntual)
        - TODO nítido (sin depth of field)
        - Como pinhole camera

        aperture = 0.05:
        - Offset pequeño
        - DOF sutil
        - Bokeh discreto

        aperture = 0.2:
        - Offset mediano
        - DOF pronunciado
        - Bokeh visible

        aperture = 0.5+:
        - Offset grande
        - Desenfoque extremo
        - Bokeh muy evidente

        aperture ↑ → Más desenfoque fuera del plano focal

    Alternativa Matemática (sin rechazo):
        ```python
        # Método polar (sin bucle)
        r = sqrt(random())      # Radio uniforme en [0,1]
        theta = random() * 2π   # Ángulo uniforme
        x = r * cos(theta)
        y = r * sin(theta)
        return Vec3(x, y, 0)
        ```

        Más eficiente (~0% rechazo) pero requiere sqrt, sin, cos.
        Para la mayoría de casos, el método de rechazo es suficiente.

    Relación con Fotografía Real:
        - Aperture (f-stop): f/1.4, f/2.8, f/5.6, etc.
        - Aperture pequeña (f/16) → TODO enfocado
        - Aperture grande (f/1.4) → Fondo desenfocado (bokeh)

        Esta función simula el efecto físico de apertura finita.
    """
    while True:
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), 0)
        if p.length() < 1.0:
            return p
