""" "
Path Tracer Cornell Box - Renderizador fotorealista basado en Monte Carlo.

Este módulo implementa un path tracer completo con las siguientes características:

Técnicas de Rendering:
    - **Path Tracing**: Integración Monte Carlo de la ecuación de renderizado
    - **Next Event Estimation (NEE)**: Muestreo directo de luces para reducir ruido
    - **Stratified Sampling**: Antialiasing con cuadrícula Sub-píxel
    - **Importance Sampling**: Muestreo hemisphere coseno-ponderado

Materiales Soportados:
    - **Difusos (Lambertian)**: Superficies mates
    - **Metales**: Reflexión especular con rugosidad (fuzz)
    - **Dieléctricos**: Vidrio/agua con refracción (Ley de Snell)
    - **Emisores**: Luces de área

Características de Cámara:
    - **Profundidad de Campo (DOF)**: Desenfoque realista (bokeh)
    - **Field of View (FOV)**: Ajustable en grados
    - **Sistema de coordenadas**: Base ortonormal configurable

Optimizaciones:
    - **BVH (Bounding Volume Hierarchy)**: Aceleración de intersecciones
    - **Renderizado Paralelo**: Multi-core con multiprocessing
    - **Gamma Correction**: Corrección 2.2 para displays

Algoritmos Físicos:
    - Ley de Snell (refracción)
    - Aproximación de Schlick (Fresnel)
    - BRDF Lambertiano
    - Ley de cosenos de Lambert

Uso:
    Ejecutar directamente para renderizar la escena Cornell Box:

    ```bash
    python main.py
    ```

    Configuración de calidad en función `render()`:
    - `samples`: 100-1000 (calidad vs tiempo)
    - `depth`: 4-12 (rebotes máximos)
    - `width/height`: Resolución de imagen

Referencias:
    - "Physically Based Rendering" (Pharr, Jakob, Humphreys)
    - "Ray Tracing in One Weekend" (Peter Shirley)
    - Cornell Box original (1984)
"""

import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere
from src.geometry import Quad
from src.geometry import BVHNode
from src.utils import generar_direccion_aleatoria
from src.utils import load_obj
from src.utils import random_in_unit_disk
import math
import random
import multiprocessing
from functools import partial

# --- FEATURE FLAG ---
USE_PARALLEL = True  # Cámbialo a False para usar un solo núcleo
# --------------------


# --- FUNCIONES MATEMÁTICAS PARA VIDRIO ---
def refract(uv, n, etai_over_etat):
    """
    Calcula el vector de refracción usando la Ley de Snell vectorial.

    Simula cómo la luz se desvía al pasar de un medio a otro (ej: aire → vidrio).

    Ley de Snell:
        η₁ sin(θ₁) = η₂ sin(θ₂)

        donde:
        - η₁, η₂ = índices de refracción (IOR) de los medios
        - θ₁ = ángulo de incidencia (desde la normal)
        - θ₂ = ángulo de refracción

    Formulación Vectorial:
        El rayo refractado se descompone en:
        1. **Componente perpendicular**: r_perp = (uv + n*cos(θ)) * η
        2. **Componente paralela**: r_parallel = n * -√(1 - |r_perp|²)

        Vector refractado final: r = r_perp + r_parallel

    Args:
        uv (Vec3): Dirección incidente NORMALIZADA (debe ser unitaria)
        n (Vec3): Normal de la superficie NORMALIZADA (debe ser unitaria)
        etai_over_etat (float): Ratio de índices de refracción (η₁/η₂)
            - Aire → Vidrio: 1.0 / 1.5 ≈ 0.667
            - Vidrio → Aire: 1.5 / 1.0 = 1.5

    Returns:
        Vec3: Vector de refracción (ya normalizado implícitamente)

    Ejemplo Numérico:
        >>> # Rayo entrando al vidrio perpendicular
        >>> uv = Vec3(0, -1, 0)  # Hacia abajo
        >>> n = Vec3(0, 1, 0)    # Normal hacia arriba
        >>> ratio = 1.0 / 1.5    # Aire → Vidrio
        >>>
        >>> refracted = refract(uv, n, ratio)
        >>> # Perpendicular → sin desviación
        >>> print(refracted)  # Vec3(0, -1, 0)
        >>>
        >>> # Rayo en ángulo
        >>> uv = Vec3(1, -1, 0).normalize()  # 45°
        >>> refracted = refract(uv, n, ratio)
        >>> # Ángulo se reduce (se acerca a la normal)

    Casos Especiales:
        - Perpendicular (θ=0°): Sin desviación
        - Paralelo (θ=90°): No puede refractar (reflexión total interna)
        - Mayor IOR → Ángulo se reduce (se acerca a normal)
        - Menor IOR → Ángulo aumenta (se aleja de normal)

    Física:
        Materiales comunes (IOR a 589nm):
        - Vacío/Aire: 1.0
        - Agua: 1.33
        - Vidrio: 1.5-1.9
        - Diamante: 2.42

    Complejidad:
        O(1) - Operaciones vectoriales constantes
    """
    cos_theta = min((uv * -1).dot(n), 1.0)
    r_out_perp = (uv + n * cos_theta) * etai_over_etat
    r_out_parallel = n * -math.sqrt(abs(1.0 - r_out_perp.length() ** 2))
    return r_out_perp + r_out_parallel


def reflectance(cosine, ref_idx):
    """
    Aproximación de Schlick para calcular reflectancia de Fresnel.

    Las ecuaciones de Fresnel describen cuánta luz se refleja vs refracta
    en una interfaz entre dos medios. Schlick es una aproximación simple y precisa.

    Fórmula de Schlick:
        R(θ) = R₀ + (1 - R₀)(1 - cos(θ))⁵

        donde:
        R₀ = ((n₁ - n₂) / (n₁ + n₂))²  (reflectancia a incidencia normal)
        θ = ángulo de incidencia

    Args:
        cosine (float): cos(θ) donde θ = ángulo entre rayo incidente y normal
            - Rango: [0, 1]
            - 1.0 = perpendicular (θ=0°)
            - 0.0 = rasante (θ=90°)
        ref_idx (float): Ratio de índices de refracción (η₁/η₂)
            - Aire → Vidrio: 1.0 / 1.5 ≈ 0.667
            - Vidrio → Aire: 1.5 / 1.0 = 1.5

    Returns:
        float: Fracción de luz reflejada [0, 1]
            - 0.0 = toda la luz refracta
            - 1.0 = toda la luz refleja (reflexión total interna)

    Ejemplo Numérico:
        >>> # Vidrio (IOR 1.5) desde el aire
        >>> ref_idx = 1.0 / 1.5
        >>>
        >>> # Perpendicular (θ=0°)
        >>> R = reflectance(1.0, ref_idx)
        >>> # R ≈ 0.04 = 4% reflejado, 96% refractado ✓
        >>>
        >>> # Ángulo rasante (θ=90°)
        >>> R = reflectance(0.0, ref_idx)
        >>> # R ≈ 1.0 = 100% reflejado ✓
        >>>
        >>> # Ángulo medio (θ=60°, cos≈0.5)
        >>> R = reflectance(0.5, ref_idx)
        >>> # R ≈ 0.09 = 9% reflejado

    Interpretación:
        - Ángulos pequeños (perpendicular): Poca reflexión (~4% vidrio)
        - Ángulos grandes (rasante): Mucha reflexión (~100%)

        Efecto visible: Mira vidrio perpendicular → transparente
                       Mira vidrio de lado → espejo

    Uso en Path Tracing:
        ```python
        R = reflectance(cos_theta, ratio)
        if random() < R:
            # Reflejar
            direction = ray.reflect(normal)
        else:
            # Refractar
            direction = refract(ray, normal, ratio)
        ```

    Física:
        Las ecuaciones de Fresnel completas son:
        - R_s = |((n₁cosθᵢ - n₂cosθₜ)/(n₁cosθᵢ + n₂cosθₜ))|²
        - R_p = |((n₁cosθₜ - n₂cosθᵢ)/(n₁cosθₜ + n₂cosθᵢ))|²
        - R = (R_s + R_p) / 2

        Schlick aproxima esto con error <1% en la mayoría de casos.

    Complejidad:
        O(1) - Operaciones aritméticas constantes
    """
    r0 = (1 - ref_idx) / (1 + ref_idx)
    r0 = r0 * r0
    return r0 + (1 - r0) * math.pow((1 - cosine), 5)


def calculate_nee(rec, world, lights):
    """
    Next Event Estimation (NEE) - Iluminación directa mediante muestreo explícito de luces.

    NEE es una técnica de reducción de varianza en path tracing. En lugar de esperar
    que los rayos aleatorios "encuentren" las luces por azar (muy ruidoso), enviamos
    rayos DIRECTAMENTE hacia cada fuente de luz y verificamos visibilidad.

    Ventajas vs Path Tracing Puro:
        - Path tracing puro: Rayos aleatorios, ~1/1000 golpean luz → MUY ruidoso
        - NEE: Rayo directo a luz cada rebote → 10-100× menos ruido

    Algoritmo:
        Para cada luz en la escena:
            1. Muestrear punto en la luz (sample_solid_angle)
            2. Lanzar shadow ray hacia ese punto
            3. Si no hay oclusión:
                a. Calcular cos(θ) (Ley de Lambert)
                b. Sumar contribución: emisión × albedo × cos(θ) × Ω/π

    Args:
        rec (HitRecord): Información del punto golpeado
            - rec.point: Posición 3D del hit
            - rec.normal: Normal de superficie (unitaria)
            - rec.color: Albedo/color del material
        world (BVHNode): Geometría de la escena (para shadow rays)
        lights (list[Quad]): Lista de objetos emisores de luz

    Returns:
        Vec3: Color de iluminación directa (RGB)

    Fórmula de Rendering:
        Para cada luz:
            L_direct = E × ρ × cos(θ) × (Ω / π)

        donde:
        - E = emisión de la luz (Vec3)
        - ρ = albedo de la superficie (Vec3)
        - cos(θ) = normal · dirección_luz (clamped a [0,1])
        - Ω = ángulo sólido subtendido por la luz
        - π = normalización de BRDF Lambertiano

    Muestreo de Ángulo Sólido:
        light.sample_solid_angle(point) retorna:
        - direction: Dirección hacia punto aleatorio en luz
        - solid_angle: Ω = (área × cos(θ_luz)) / distancia²

        Esto es importance sampling: más muestras donde la luz es más brillante.

    Shadow Ray:
        ```python
        shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)
        ```
        - Offset 0.001: Evita "shadow acne" (self-intersection)
        - Dirección l_dir: Hacia punto en luz
        - Rango: [0.001, distancia_luz - 0.001]

    Ejemplo Numérico:
        ```python
        # Punto en superficie horizontal
        rec.point = Vec3(0, 0, 0)
        rec.normal = Vec3(0, 1, 0)  # Arriba
        rec.color = Vec3(0.8, 0.8, 0.8)  # Gris claro

        # Luz en el techo
        light.center = Vec3(0, 10, 0)
        light.emission = Vec3(100, 100, 100)
        light.area = 1.0

        # Cálculo:
        distance = 10.0
        cos_theta_surface = 1.0  # Perpendicular
        cos_theta_light = 1.0    # Luz apunta abajo
        solid_angle = (1.0 * 1.0) / (10.0²) = 0.01

        L_direct = Vec3(100,100,100) * Vec3(0.8,0.8,0.8) * 1.0 * (0.01 / π)
                 ≈ Vec3(0.255, 0.255, 0.255)
        ```

    Casos Especiales:
        - Luz ocluida: No contribuye (shadow ray bloqueado)
        - cos(θ) < 0: Luz desde abajo, no contribuye (clamped a 0)
        - Múltiples luces: Se suman todas las contribuciones

    Optimización:
        - Solo se muestrea 1 punto por luz (no Monte Carlo completo)
        - Para luces grandes, aumentar muestras reduce ruido adicional

    Complejidad:
        O(L × log(N)) donde:
        - L = número de luces
        - N = número de objetos (hit via BVH)
    """
    direct_light = Vec3(0, 0, 0)
    # lights is passed as argument now

    for light in lights:
        # 1. Obtenemos dirección y el ángulo sólido que ocupa la luz
        l_dir, solid_angle = light.sample_solid_angle(rec.point)

        # 2. Shadow Ray: Verificamos visibilidad
        shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)

        # Verificamos si golpeamos la luz directamente sin obstáculos
        # Usamos el BVH (world) para buscar intersecciones eficientemente
        distancia_a_luz = (light.center - rec.point).length()
        h = world.hit(shadow_ray, 0.001, distancia_a_luz - 0.001)

        # Verificamos si lo que golpeamos es la luz misma
        if h and h.obj_ref == light:
            # 3. Cálculo de Iluminancia
            # cos_theta: Cuánta luz recibe la superficie según su inclinación
            cos_theta = max(0, rec.normal.dot(l_dir))

            # Con Muestreo de Ángulo Sólido, la fórmula es:
            # Luz = Emisión * Color * cos(theta) * (Ángulo Sólido / PI)
            # El PI viene de la normalización del material difuso (Lambert)
            direct_light = direct_light + (
                light.emission * rec.color * (cos_theta * solid_angle / math.pi)
            )

    return direct_light


def color_ray(ray, world, lights, depth, puede_ver_luz=True):
    """
    Función recursiva de Path Tracing - Calcula el color llegando por un rayo.

    Implementa la ecuación de renderizado mediante integración Monte Carlo:

        L_out(x, ω_out) = L_e(x, ω_out) + ∫_Ω f_r(x, ω_in, ω_out) L_in(x, ω_in) cos(θ) dω_in

    donde:
    - L_out = luz saliente (lo que vemos)
    - L_e = luz emitida (si es una luz)
    - f_r = BRDF (función de reflexión bidireccional)
    - L_in = luz entrante (recursión)
    - Ω = hemisferio sobre la superficie
    - θ = ángulo entre ω_in y la normal

    Algoritmo (Path Tracing + NEE):
        1. Lanzar rayo, encontrar intersección más cercana
        2. Si es luz: Retornar emisión (si puede_ver_luz=True)
        3. Si es metal: Reflejar especularmente con rugosidad
        4. Si es vidrio: Reflejar/refractar según Fresnel
        5. Si es difuso:
            a. NEE: Iluminación directa de luces
            b. Rebote: Dirección aleatoria en hemisferio (luz indirecta)
        6. Recursión hasta depth=0

    Args:
        ray (Ray): Rayo a trazar (origen + dirección normalizada)
        world (BVHNode): Estructura de aceleración con toda la geometría
        lights (list): Lista de objetos emisores (para NEE)
        depth (int): Profundidad de recursión restante (0 = terminar)
        puede_ver_luz (bool): Si False, no renderiza luces directamente
            - True: Rayos especulares (metales, vidrio)
            - False: Rayos difusos (evita double-counting con NEE)

    Returns:
        Vec3: Color RGB acumulado a lo largo del camino del rayo

    Materiales Implementados:

        **Difuso (Lambertian)**:
            - BRDF constante: f_r = ρ / π
            - Scatter: Dirección aleatoria en hemisferio
            - Iluminación: NEE (directa) + Rebote (indirecta)

        **Metal**:
            - Reflexión especular: R = I - 2(I·N)N
            - Rugosidad (fuzz): Perturbar dirección con vector aleatorio
            - Solo luz especular (puede_ver_luz=True en recursión)

        **Vidrio/Dieléctrico**:
            - Refracción: Ley de Snell vectorial
            - Reflexión: Fresnel-Schlick (probabilidad)
            - Índice de refracción (IOR): Típicamente 1.5 para vidrio

    Ejemplo Recursión:
        ```
        Cámara → Rayo1 (depth=10)
                  ↓
                Esfera difusa roja
                  ↓
                NEE: Luz directa = rojo × luz × cos(θ)
                  ↓
                Rebote: Rayo2 (depth=9) dirección aleatoria
                  ↓
                Pared blanca difusa
                  ↓
                NEE: Luz directa = blanco × luz × cos(θ)
                  ↓
                Rebote: Rayo3 (depth=8) ...
                  ↓
                ... hasta depth=0 → Vec3(0,0,0)

        Color final = rojo × (NEE1 + blanco × (NEE2 + ...))
        ```

    Casos Base:
        - depth <= 0: Retorna negro (termina recursión)
        - No intersección: Retorna color de cielo/ambiente
        - Rayo dentro de superficie (dot < 0): Absorción

    Optimizaciones:
        - Shadow acne prevention: offset 0.001 en rebotes
        - Russian roulette: Terminar caminos oscuros probabilísticamente (no implementado)
        - Importancesampling: Hemisferio coseno-ponderado (parcial)

    Parámetros de Calidad:
        - depth bajo (4): Rápido, poca iluminación indirecta
        - depth medio (8): Balance
        - depth alto (16+): Lento, convergencia completa

    Complejidad:
        O(depth × log(N)) en promedio, donde N = objetos en escena
        (BVH reduce de O(N) a O(log N) por intersección)
    """
    if depth <= 0:
        return Vec3(0, 0, 0)

    # El BVHNode se encarga de encontrar el choque más cercano eficientemente
    closest_hit = world.hit(ray, 0.001, float("inf"))

    if closest_hit:
        # Si el rayo viene directo de la cámara y choca con la luz, la dibujamos
        # 1. Luz Directa (Emisores)
        if closest_hit.emission.length() > 0:
            return closest_hit.emission if puede_ver_luz else Vec3(0, 0, 0)

        # 2. Materiales Metálicos
        if closest_hit.is_metal:
            # --- LÓGICA DE METAL ---
            # 1. Calculamos la reflexión perfecta
            reflected_direction = ray.direction.normalize().reflect(closest_hit.normal)

            # 2. Aplicamos rugosidad: sumamos un vector aleatorio dentro de una esfera de radio 'fuzz'
            # Usamos la función que ya tienes para direcciones aleatorias
            perturbacion = (
                generar_direccion_aleatoria(closest_hit.normal) * closest_hit.fuzz
            )

            final_direction = (reflected_direction + perturbacion).normalize()

            # Verificamos que el rayo no se haya metido dentro de la esfera por la rugosidad
            if final_direction.dot(closest_hit.normal) > 0:
                # Añadimos un pequeño margen (0.001) para evitar que el rayo choque con la misma esfera
                scattered_ray = Ray(
                    closest_hit.point + closest_hit.normal * 0.001, final_direction
                )
                # ¡IMPORTANTE!: puede_ver_luz = True porque es un reflejo especular
                return (
                    color_ray(scattered_ray, world, lights, depth - 1, True)
                    * closest_hit.color
                )
            else:
                return Vec3(0, 0, 0)
        # 3. Materiales Dieléctricos (VIDRIO)
        elif closest_hit.is_dielectric:
            # Determinamos si el rayo entra o sale del objeto
            en_frente = ray.direction.dot(closest_hit.normal) < 0
            ratio = (1.0 / closest_hit.ior) if en_frente else closest_hit.ior
            normal = closest_hit.normal if en_frente else closest_hit.normal * -1

            unit_dir = ray.direction.normalize()
            cos_theta = min((unit_dir * -1).dot(normal), 1.0)
            sin_theta = math.sqrt(1.0 - cos_theta**2)

            # ¿Reflexión Interna Total o Refracción?
            no_puede_refractar = ratio * sin_theta > 1.0
            if no_puede_refractar or reflectance(cos_theta, ratio) > random.random():
                direccion = unit_dir.reflect(normal)
            else:
                direccion = refract(unit_dir, normal, ratio)

            scattered_ray = Ray(closest_hit.point + direccion * 0.001, direccion)
            return (
                color_ray(scattered_ray, world, lights, depth - 1, True)
                * closest_hit.color
            )
        else:
            # A. LUZ DIRECTA (NEE)
            luz_directa = calculate_nee(closest_hit, world, lights)

            # B. LUZ INDIRECTA (Rebote aleatorio)
            target = closest_hit.point + generar_direccion_aleatoria(closest_hit.normal)
            new_ray = Ray(
                closest_hit.point + closest_hit.normal * 0.001,
                target - closest_hit.point,
            )

            # IMPORTANTE: puede_ver_luz=False para que los rebotes no vuelvan a sumar la luz
            luz_indirecta = (
                color_ray(new_ray, world, lights, depth - 1, False) * closest_hit.color
            )

            return luz_directa + luz_indirecta

    # Fondo (Cielo)
    if ray.direction.y > 0.8:
        return Vec3(1.5, 1.5, 1.5)
    return Vec3(0.05, 0.05, 0.05)


def cornell_box():
    """
    [DEPRECADA] Usa render_obj(mode="spheres") en su lugar.

    Crea la escena clásica Cornell Box para testing de path tracers.

    La Cornell Box es un estándar de la industria creado en 1984 por el Cornell
    Program of Computer Graphics para validar algoritmos de iluminación global.

    Características de la Escena:
        - Dimensiones: 555 × 555 × 555 unidades
        - 5 paredes difusas (piso, techo, pared trasera + 2 laterales coloreadas)
        - 1 luz de área en el techo
        - 2 objetos: 1 esfera de vidrio + 1 esfera metálica

    Configuración de Materiales:
        - Pared izquierda: Verde (RGB 0.12, 0.45, 0.15)
        - Pared derecha: Roja (RGB 0.65, 0.05, 0.05)
        - Otras paredes: Blanco difuso (RGB 0.73, 0.73, 0.73)
        - Luz: Emisión blanca intensa (RGB 15, 15, 15)

    Geometría:
        Todas las paredes son Quads (cuadriláteros) definidos por:
        - Q: Esquina inicial
        - u: Vector lado 1
        - v: Vector lado 2

        Normales apuntan HACIA ADENTRO de la caja.

    Objetos:
        1. **Esfera de vidrio** (dieléctrico):
           - Centro: (190, 90, 190)
           - Radio: 90
           - IOR: 1.5 (vidrio estándar)

        2. **Esfera metálica** (espejo perfecto):
           - Centro: (400, 90, 370)
           - Radio: 90
           - Fuzz: 0.0 (reflexión perfecta)

    Luz:
        - Posición: Centro del techo (ligeramente offset)
        - Tamaño: 130 × 105 unidades
        - Tipo: Quad emisor

    Returns:
        BVHNode: Árbol BVH con toda la geometría de la escena

    Uso:
        ```python
        scene = cornell_box()
        hit = scene.hit(ray, 0.001, float('inf'))
        ```

    Efectos Visuales Esperados:
        - Caustics: Luz focalizada por esfera de vidrio
        - Color bleeding: Verde/rojo sangran en paredes blancas
        - Soft shadows: Sombras suaves por luz de área
        - Specular reflections: Esfera metálica refleja escena
        - Refraction: Vidrio distorsiona objetos detrás

    Historial:
        La Cornell Box original tenía dos cajas en vez de esferas.
        Esta versión usa esferas para mostrar materiales complejos.

    Complejidad:
        Construcción: O(N log N) donde N = número de objetos (BVH)
        Memoria: O(N)
    """
    lista = []

    # Materiales de las paredes
    rojo = Vec3(0.65, 0.05, 0.05)
    blanco = Vec3(0.73, 0.73, 0.73)
    verde = Vec3(0.12, 0.45, 0.15)
    luz = Vec3(15, 15, 15)

    # Paredes (Q, u, v, color)
    lista.append(
        Quad(Vec3(555, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), verde)
    )  # Izquierda
    lista.append(Quad(Vec3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), rojo))  # Derecha
    lista.append(Quad(Vec3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), blanco))  # Piso
    lista.append(
        Quad(Vec3(555, 555, 555), Vec3(-555, 0, 0), Vec3(0, 0, -555), blanco)
    )  # Techo
    lista.append(
        Quad(Vec3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), blanco)
    )  # Fondo

    # Luz de techo (pequeño Quad brillante)
    lista.append(
        Quad(
            Vec3(213, 554, 227),
            Vec3(130, 0, 0),
            Vec3(0, 0, 105),
            Vec3(0, 0, 0),
            emission=luz,
        )
    )

    # Agregamos tus esferas favoritas dentro
    lista.append(
        Sphere(Vec3(190, 90, 190), 90, Vec3(1, 1, 1), is_dielectric=True, ior=1.5)
    )  # Vidrio
    lista.append(
        Sphere(Vec3(400, 90, 370), 90, Vec3(1, 1, 1), is_metal=True, fuzz=0.0)
    )  # Metal

    return BVHNode.create(lista)


def render_row(y, width, height, samples, depth, world, lights, camera_params):
    """
    Renderiza una fila completa de la imagen (diseñada para ejecución paralela).

    Esta función es una "unidad de trabajo" independiente que puede ejecutarse
    en paralelo en múltiples núcleos de CPU mediante multiprocessing.Pool.

    Estrategia de Antialiasing - Stratified Sampling:
        En lugar de muestras completamente aleatorias, dividimos cada píxel en
        una cuadrícula regular y tomamos 1 muestra por celda:

        ```
        Píxel dividido en 10×10 (samples=100):
        +--+--+--+--+
        |• |  •|•  | ...  (1 muestra aleatoria por celda)
        +--+--+--+--+
        | •| • | •| ...
        +--+--+--+--+
        ```

        Ventajas vs muestreo puro:
        - Mejor distribución (sin clustering)
        - Menos ruido para el mismo número de muestras

    Depth of Field (Bokeh):
        Para cada muestra, se genera un offset aleatorio en la "lente":
        1. random_in_unit_disk() × lens_radius → offset en plano de lente
        2. Origen desplazado: origin + offset
        3. Dirección ajustada para pasar por plano de enfoque

        Resultado: Objetos en plano focal → nítidos
                  Objetos fuera de foco → borrosos (bokeh)

    Args:
        y (int): Índice de la fila a renderizar [0, height-1]
        width (int): Ancho de la imagen en píxeles
        height (int): Alto de la imagen en píxeles
        samples (int): Muestras por píxel (típicamente 100-1000)
            - Debe ser cuadrado perfecto (100, 400, 900...) para cuadrícula
        depth (int): Profundidad máxima de recursión (rebotes)
        world (BVHNode): Geometría de la escena
        lights (list): Lista de luces para NEE
        camera_params (dict): Parámetros de cámara:
            - 'origin': Posición de la cámara
            - 'lower_left': Esquina inferior izquierda del viewport
            - 'horizontal': Vector horizontal del viewport
            - 'vertical': Vector vertical del viewport
            - 'u', 'v': Vectores base de la cámara
            - 'lens_radius': Radio de apertura (DOF)

    Returns:
        list[list[int]]: Lista de píxeles RGB para esta fila
            - Formato: [[r1,g1,b1], [r2,g2,b2], ...]
            - Valores: 0-255 (uint8)

    Proceso por Píxel:
        1. Inicializar acumulador de color: col = Vec3(0,0,0)
        2. Para cada muestra en cuadrícula stratified:
            a. Calcular offset sub-píxel (s, t) ∈ [0,1]
            b. Generar offset DOF en lente
            c. Calcular origen y dirección de rayo
            d. Trazar rayo: col += color_ray(...)
        3. Promediar: pixel_color = col / num_samples
        4. Gamma correction: RGB^(1/2.2)
        5. Cuantizar a [0, 255]

    Gamma Correction:
        Displays asumen entrada en espacio gamma (no lineal).
        Debemos convertir desde espacio lineal:

        ```python
        # Espacio lineal → Espacio gamma
        rgb_display = rgb_linear^(1/gamma)

        # Típicamente gamma = 2.2 (sRGB aproximado)
        ```

        Sin gamma: Imagen muy oscura
        Con gamma 2.2: Imagen correcta ✓

    Ejemplo Numérico:
        ```python
        # Pixel acumula de 100 muestras
        col = Vec3(0.5, 0.3, 0.1)  # Promedio lineal

        # Gamma correction (2.2):
        r = (0.5)^(1/2.2) ≈ 0.73
        g = (0.3)^(1/2.2) ≈ 0.58
        b = (0.1)^(1/2.2) ≈ 0.35

        # Cuantizar a [0,255]:
        r_final = int(255.99 × 0.73) = 186
        g_final = int(255.99 × 0.58) = 148
        b_final = int(255.99 × 0.35) = 89
        ```

    Paralelización:
        ```python
        # Distribuir filas entre núcleos
        with Pool(8) as pool:
            rows = pool.map(render_row, range(height))

        # Cada núcleo procesa filas independientes
        # Núcleo 0: filas 0, 8, 16, ...
        # Núcleo 1: filas 1, 9, 17, ...
        # ...
        ```

    Optimización:
        - Una fila completa es suficiente trabajo para amortizar overhead
        - Si width muy pequeño, considerar agrupar varias filas

    Complejidad:
        O(width × samples × depth × log(N))
        donde N = objetos en escena
    """
    row_pixels = []
    # Calculamos la raíz de las muestras para crear una cuadrícula (ej: sqrt(100) = 10)
    s_side = int(math.sqrt(samples))

    origin = camera_params["origin"]
    lower_left = camera_params["lower_left"]
    horizontal = camera_params["horizontal"]
    vertical = camera_params["vertical"]
    u_cam = camera_params["u"]
    v_cam = camera_params["v"]
    lens_radius = camera_params["lens_radius"]

    for x in range(width):
        col = Vec3(0, 0, 0)
        # Bucle de Antialiasing Estratificado (Cuadrícula)
        for i in range(s_side):
            for j in range(s_side):
                # Dividimos el píxel en sub-celdas y lanzamos un rayo en cada una
                u_offset = (i + random.random()) / s_side
                v_offset = (j + random.random()) / s_side

                # Calculamos las coordenadas normalizadas (0 a 1)
                s = (x + u_offset) / width
                t = (
                    y + v_offset
                ) / height  # En trazado de rayos 't' suele ser vertical

                # --- LÓGICA BOKEH ---
                # 1. Obtenemos un desplazamiento aleatorio en la lente
                rd = random_in_unit_disk() * lens_radius
                offset = u_cam * rd.x + v_cam * rd.y

                # 2. El rayo sale desde el origen desplazado
                new_origin = origin + offset
                # 3. La dirección apunta al punto en el plano de enfoque
                direction = lower_left + horizontal * s + vertical * t - new_origin

                # El rayo ahora se calcula basado en el plano de la cámara
                # Dirección = Punto en el plano - Origen
                ray = Ray(new_origin, direction.normalize())
                col = col + color_ray(ray, world, lights, depth)

        # Promediamos por el total real de muestras (s_side * s_side)
        pixel_color = col / (s_side * s_side)

        # Corrección gamma (Gamma 2.2) y clamping a [0, 255]
        # Usamos max(0, val) para evitar errores con valores negativos flotantes muy pequeños
        r = min(255, int(255.99 * math.pow(max(0, pixel_color.x), 1 / 2.2)))
        g = min(255, int(255.99 * math.pow(max(0, pixel_color.y), 1 / 2.2)))
        b = min(255, int(255.99 * math.pow(max(0, pixel_color.z), 1 / 2.2)))
        row_pixels.append([r, g, b])

    return row_pixels


def render():
    """
    Función principal de renderizado - Configura y ejecuta el path tracer completo.

    Esta función orquesta todo el proceso de rendering:
    1. Configurar parámetros de calidad (resolución, samples, depth)
    2. Configurar cámara (posición, FOV, DOF)
    3. Cargar escena (Cornell Box)
    4. Renderizar en paralelo o secuencialmente
    5. Guardar imagen final

    Parámetros de Calidad:

        **width, height** (int):
            - 400×400: Rápido, preview (~2 min)
            - 800×800: Balance (~10 min)
            - 1920×1080: Alta calidad (~1 hora)

        **samples** (int):
            - 100: Muy ruidoso, preview rápido
            - 400: Ruido aceptable (CONFIGURACIÓN ACTUAL)
            - 1000: Limpio, producción
            - 4000+: Muy limpio, render final

            Tiempo ∝ samples (lineal)

        **depth** (int):
            - 4: Solo 4 rebotes, iluminación simple
            - 8: Balance (CONFIGURACIÓN ACTUAL)
            - 12: Iluminación indirecta completa
            - 16+: Convergencia total (marginal benefit)

            Tiempo ∝ depth (lineal aprox)

    Configuración de Cámara:

        **Posición y Orientación**:
            ```python
            camera_origin = Vec3(278, 278, -800)
            lookat = Vec3(278, 278, 278)  # Centro Cornell Box
            vup = Vec3(0, 1, 0)           # Vector "arriba"
            ```

        **Field of View (FOV)**:
            - 40°: Visión estrecha (telefoto) - CONFIGURACIÓN ACTUAL
            - 60°: Visión normal (similar a ojo humano)
            - 90°: Visión amplia (gran angular)

        **Depth of Field**:
            - aperture = 0.0: Todo enfocado (pinhole)
            - aperture = 20.0: Bokeh evidente (CONFIGURACIÓN ACTUAL)
            - aperture = 50.0: Desenfoque extremo

            - dist_to_focus: Distancia al plano nítido
              (Auto-calculada como distancia a 'lookat')

    Sistema de Coordenadas de Cámara:

        Se construye base ortonormal {u, v, w}:

        ```
        w = (origin - lookat).normalize()  # Hacia atrás
        u = vup.cross(w).normalize()       # Derecha
        v = w.cross(u)                     # Arriba (recomputado)

              v (up)
              ↑
              |
        w ←---•  (cámara)
             /
            u (right)
        ```

    Viewport y Proyección:

        ```python
        # Tamaño del plano de proyección
        theta = radians(fov)
        h = tan(theta / 2)
        viewport_height = 2.0 * h
        viewport_width = aspect_ratio * viewport_height

        # Escalado por distancia de enfoque
        horizontal = u * viewport_width * dist_to_focus
        vertical = v * viewport_height * dist_to_focus

        # Esquina inferior izquierda
        lower_left = origin - horizontal/2 - vertical/2 - w*dist_to_focus
        ```

    Renderizado Paralelo:

        Si USE_PARALLEL = True:
        ```python
        num_cores = CPU count (ej: 8)
        Pool.map(render_row, [0,1,2,...,height-1])

        # Speedup teórico: ~8× en CPU de 8 núcleos
        # Speedup real: ~6-7× (overhead comunicación)
        ```

        Si USE_PARALLEL = False:
        ```python
        for y in range(height):
            row = render_row(y, ...)

        # Útil para debugging (no paraleliza errores)
        ```

    Post-procesamiento:

        1. **Flip vertical**: `np.flipud(data)`
           - Coordenadas de imagen: Y=0 arriba
           - Coordenadas de rendering: Y=0 abajo

        2. **Gamma correction**: Ya aplicada en render_row()

        3. **Guardar**: PNG sin compresión (PIL)

    Salida:

        Archivo: `output/bokeh.png`
        Formato: PNG RGB 8-bit
        Espacio de color: sRGB (aprox, gamma 2.2)

    Uso:
        ```python
        # Ejecutar desde línea de comandos
        python main.py

        # O importar
        from main import render
        render()
        ```

    Complejidad:
        O(width × height × samples × depth × log(N))
        donde N = objetos en escena
    """
    # Para la Cornell Box es mejor un aspecto cuadrado
    width, height = 400, 400
    samples = 400
    depth = 8

    # --- CONFIGURACIÓN DE CÁMARA (Para la caja de 555 unidades) ---
    camera_origin = Vec3(278, 278, -800)
    lookat = Vec3(278, 278, 278)  # Apuntamos al centro de la caja
    vup = Vec3(0, 1, 0)

    # Ahora calculamos la distancia focal automáticamente o la definimos
    dist_to_focus = (camera_origin - lookat).length()
    fov = 40.0  # Grados
    aperture = 20.0  # Se sube este valor para ver más desenfoque
    lens_radius = aperture / 2

    # Matemática de la cámara (Proyección)
    aspect_ratio = width / height
    theta = math.radians(fov)
    h = math.tan(theta / 2)
    viewport_height = 2.0 * h
    viewport_width = aspect_ratio * viewport_height

    # Construimos la base ortonormal de la cámara
    w = (camera_origin - lookat).normalize()
    u = vup.cross(w).normalize()
    v = w.cross(u)

    # Escalamos los vectores horizontales y verticales por la distancia de enfoque
    horizontal = u * viewport_width * dist_to_focus
    vertical = v * viewport_height * dist_to_focus

    # Esquina inferior izquierda (ahora depende de dist_to_focus)
    lower_left = camera_origin - horizontal / 2 - vertical / 2 - w * dist_to_focus

    camera_params = {
        "origin": camera_origin,
        "lower_left": lower_left,
        "horizontal": horizontal,
        "vertical": vertical,
        "u": u,
        "v": v,
        "lens_radius": lens_radius,  # Necesario para el Paso 3
    }

    # Cargamos la Cornell Box (modo bunny por defecto)
    world, lights = render_obj(mode="bunny")

    if USE_PARALLEL:
        num_cores = multiprocessing.cpu_count()
        print(f"Iniciando render paralelo con {num_cores} núcleos...")

        # Preparamos la función con los argumentos constantes
        worker_func = partial(
            render_row,
            width=width,
            height=height,
            samples=samples,
            depth=depth,
            world=world,
            lights=lights,
            camera_params=camera_params,
        )

        # Creamos un Pool de procesos
        with multiprocessing.Pool(processes=num_cores) as pool:
            # pool.map distribuye las filas y mantiene el orden original
            results = pool.map(worker_func, range(height))

        # Convertimos la lista de filas en el array final de la imagen
        data = np.array(results, dtype=np.uint8)
    else:
        print("Iniciando render secuencial (un solo núcleo)...")
        data = np.zeros((height, width, 3), dtype=np.uint8)

        for y in range(height):
            print(f"Progreso: {int(y/height*100)}%", end="\r")

            row_data = render_row(
                y, width, height, samples, depth, world, lights, camera_params
            )
            data[y] = row_data

    data = np.flipud(data)
    Image.fromarray(data).save("output/bokeh.png")
    print("\n¡Render finalizado!")


def render_obj(mode="bunny"):
    """
    Configura y carga la escena a renderizar (Cornell Box con diferentes objetos).

    Esta función centraliza la construcción de escenas, permitiendo diferentes
    configuraciones mediante el parámetro 'mode'. La Cornell Box (paredes y luz)
    se mantiene igual en todos los modos, solo cambian los objetos del centro.

    Modos Disponibles:
        - "spheres": Cornell Box con 2 esferas (vidrio + metal)
        - "bunny": Cornell Box con modelo 3D bunny.obj de vidrio (default)

    Args:
        mode (str): Tipo de objetos a renderizar en el centro
            - "spheres": 1 esfera de vidrio + 1 esfera metálica
            - "bunny": Modelo 3D OBJ (bunny.obj) de vidrio

    Returns:
        tuple[BVHNode, list]:
            - world: BVH con toda la geometría
            - lights: Lista de objetos emisores (para NEE)

    Configuración de Materiales:

        **Paredes** (Idénticas en todos los modos):
            - Izquierda (Verde): RGB(0.12, 0.45, 0.15)
            - Derecha (Rojo): RGB(0.65, 0.05, 0.05)
            - Resto (Blanco): RGB(0.73, 0.73, 0.73)

        **Luz** (Idéntica en todos los modos):
            - Emisión: RGB(15, 15, 15) para spheres, RGB(40, 40, 40) para bunny
            - Posición: Cerca del techo (Y=554 o Y=554.9)
            - Tamaño: 130 × 105 unidades

        **Modo "spheres"**:
            - Esfera de vidrio:
              * Centro: (190, 90, 190)
              * Radio: 90
              * Material: Vidrio (IOR 1.5)
            - Esfera metálica:
              * Centro: (400, 90, 370)
              * Radio: 90
              * Material: Metal (fuzz 0.0 = espejo perfecto)

        **Modo "bunny"**:
            - Material: Vidrio (IOR 1.5)
            - Escala: 3000× (bunny.obj original es muy pequeño)
            - Offset: Centrado en el piso (278, 0, 278)
            - Triángulos: ~69,000

    Geometría de Paredes (Quads):

        Importante: Las normales apuntan HACIA ADENTRO de la caja.

        ```python
        # Pared izquierda (Verde, normal apunta a +X)
        Quad(Q=Vec3(555,0,0), u=Vec3(0,555,0), v=Vec3(0,0,555), color=verde)

        # Normal = u × v / |u × v|
        # u × v = Vec3(0,555,0) × Vec3(0,0,555)
        #       = Vec3(-555², 0, 0)  → normalizado → Vec3(-1, 0, 0)
        # Apunta hacia DENTRO ✓
        ```

        Si las normales apuntaran hacia fuera, la caja sería negra (backface culling).

    Modelo 3D (.OBJ):

        ```python
        modelo_triangulos = load_obj(
            "models/bunny.obj",
            color=Vec3(0.9, 0.9, 0.9),
            offset=Vec3(278, 0, 278),  # Centrado
            scale=3000.0,               # CRÍTICO: escala grande
            material_params={'is_dielectric': True, 'ior': 1.5}
        )
        ```

        El bunny.obj de Stanford tiene ~69K triángulos.
        Sin BVH: ~300 horas renderizar
        Con BVH: ~2 minutos ✓ (aceleración 9000×)

    Extracción de Luces:

        ```python
        lights = [obj for obj in lista_objetos if obj.emission.length() > 0]
        ```

        Solo objetos con emisión > 0 se consideran luces para NEE.
        Típicamente 1 luz (techo) en Cornell Box.

    BVH (Bounding Volume Hierarchy):

        Construcción:
        ```python
        world = BVHNode.create(lista_objetos)
        ```

        Resultado: Árbol binario donde:
        - Nodos internos: AABB (bounding boxes)
        - Hojas: Geometría primitiva (Quad, Triangle, Sphere)

        Beneficio:
        - Sin BVH: O(N) intersecciones por rayo
        - Con BVH: O(log N) intersecciones por rayo

        Para N=69,000 triángulos:
        - Sin BVH: 69,000 tests/rayo
        - Con BVH: ~16 tests/rayo ✓

    Ejemplo de Uso:
        ```python
        # Cornell Box con esferas
        world, lights = render_obj(mode="spheres")

        # Cornell Box con bunny
        world, lights = render_obj(mode="bunny")

        # Trazar rayo
        ray = Ray(origin, direction)
        hit = world.hit(ray, 0.001, float('inf'))

        # NEE
        if hit:
            direct = calculate_nee(hit, world, lights)
        ```

    Extensibilidad:

        Para añadir nuevos objetos en la Cornell Box:
        ```python
        def render_obj(mode="bunny"):
            # ... crear paredes y luz ...

            if mode == "spheres":
                # ... esferas ...
            elif mode == "bunny":
                # ... bunny ...
            elif mode == "custom":
                # Tu objeto personalizado
                lista_objetos.extend([...])

            world = BVHNode.create(lista_objetos)
            lights = [obj for obj in lista_objetos if obj.emission.length() > 0]
            return world, lights
        ```

    Complejidad:
        - Construcción BVH: O(N log N)
        - Memoria: O(N)
        donde N = número total de primitivas
    """
    lista_objetos = []

    # Materiales (idénticos para todos los modos)
    rojo = Vec3(0.65, 0.05, 0.05)
    blanco = Vec3(0.73, 0.73, 0.73)
    verde = Vec3(0.12, 0.45, 0.15)

    # PAREDES DE LA CORNELL BOX (idénticas para todos los modos)
    # Normales apuntan HACIA ADENTRO

    # Izquierda (Verde)
    lista_objetos.append(Quad(Vec3(555, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), verde))
    # Derecha (Rojo)
    lista_objetos.append(Quad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), rojo))
    # Piso (Blanco) - Normal hacia arriba (0, 1, 0)
    lista_objetos.append(Quad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(555, 0, 0), blanco))
    # Techo (Blanco) - Normal hacia abajo (0, -1, 0)
    lista_objetos.append(
        Quad(Vec3(555, 555, 555), Vec3(0, 0, -555), Vec3(-555, 0, 0), blanco)
    )
    # Fondo (Blanco) - Normal hacia la cámara (0, 0, -1)
    lista_objetos.append(
        Quad(Vec3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), blanco)
    )

    # OBJETOS DEL CENTRO (cambian según el modo)
    if mode == "spheres":
        # Modo esferas: 2 esferas (vidrio + metal)
        luz_emision = Vec3(15, 15, 15)

        # Esfera de vidrio (dieléctrico)
        lista_objetos.append(
            Sphere(Vec3(190, 90, 190), 90, Vec3(1, 1, 1), is_dielectric=True, ior=1.5)
        )

        # Esfera metálica (espejo perfecto)
        lista_objetos.append(
            Sphere(Vec3(400, 90, 370), 90, Vec3(1, 1, 1), is_metal=True, fuzz=0.0)
        )

        # LUZ DE TECHO (para modo esferas)
        lista_objetos.append(
            Quad(
                Vec3(213, 554, 227),
                Vec3(130, 0, 0),
                Vec3(0, 0, 105),
                Vec3(0, 0, 0),
                emission=luz_emision,
            )
        )

    elif mode == "bunny":
        # Modo bunny: Modelo 3D OBJ
        luz_emision = Vec3(40, 40, 40)

        # Modelo 3D - Bunny de vidrio
        modelo_triangulos = load_obj(
            "models/bunny.obj",
            color=Vec3(0.9, 0.9, 0.9),
            offset=Vec3(278, 0, 278),  # Centrado en el piso
            scale=3000.0,
            material_params={
                "is_dielectric": True,
                "ior": 1.5,
            },
        )
        lista_objetos.extend(modelo_triangulos)

        # LUZ DE TECHO (para modo bunny - más intensa)
        lista_objetos.append(
            Quad(
                Vec3(213, 554.9, 227),
                Vec3(130, 0, 0),
                Vec3(0, 0, 105),
                Vec3(0, 0, 0),
                emission=luz_emision,
            )
        )
    else:
        raise ValueError(f"Modo '{mode}' no reconocido. Usa 'spheres' o 'bunny'.")

    # Construir BVH y extraer luces
    world = BVHNode.create(lista_objetos)
    lights = [obj for obj in lista_objetos if obj.emission.length() > 0]
    return world, lights


if __name__ == "__main__":
    render()
