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
    """Calcula el vector de refracción usando la Ley de Snell."""
    cos_theta = min((uv * -1).dot(n), 1.0)
    r_out_perp = (uv + n * cos_theta) * etai_over_etat
    r_out_parallel = n * -math.sqrt(abs(1.0 - r_out_perp.length() ** 2))
    return r_out_perp + r_out_parallel


def reflectance(cosine, ref_idx):
    """Aproximación de Schlick para determinar la probabilidad de reflexión."""
    r0 = (1 - ref_idx) / (1 + ref_idx)
    r0 = r0 * r0
    return r0 + (1 - r0) * math.pow((1 - cosine), 5)


def calculate_nee(rec, world, lights):
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
    Función que procesa una sola fila de la imagen.
    Esta función es la que se distribuirá entre los núcleos.
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
                t = (y + v_offset) / height  # En trazado de rayos 't' suele ser vertical

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
    # Para la Cornell Box es mejor un aspecto cuadrado
    width, height = 400, 400
    samples = 400
    depth = 8

    # --- CONFIGURACIÓN DE CÁMARA (Para la caja de 555 unidades) ---
    camera_origin = Vec3(278, 278, -800)
    lookat = Vec3(278, 278, 278) # Apuntamos al centro de la caja
    vup = Vec3(0, 1, 0)

    # Ahora calculamos la distancia focal automáticamente o la definimos
    dist_to_focus = (camera_origin - lookat).length()
    fov = 40.0  # Grados
    aperture = 20.0 # Se sube este valor para ver más desenfoque
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
        "lens_radius": lens_radius # Necesario para el Paso 3
    }

    # Cargamos la Cornell Box
    world, lights = render_obj(mode="cornell")

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


def render_obj(mode="cornell"):
    lista_objetos = []
    if mode == "cornell":
        rojo = Vec3(0.65, 0.05, 0.05)
        blanco = Vec3(0.73, 0.73, 0.73)
        verde = Vec3(0.12, 0.45, 0.15)
        luz_emision = Vec3(40, 40, 40)

        # PAREDES (Ajustadas para que las normales apunten HACIA ADENTRO)
        # Izquierda (Verde)
        lista_objetos.append(Quad(Vec3(555,0,0), Vec3(0,555,0), Vec3(0,0,555), verde))
        # Derecha (Rojo)
        lista_objetos.append(Quad(Vec3(0,0,0), Vec3(0,0,555), Vec3(0,555,0), rojo))
        # Piso (Blanco) - Normal hacia arriba (0, 1, 0)
        lista_objetos.append(Quad(Vec3(0,0,0), Vec3(0,0,555), Vec3(555,0,0), blanco))
        # Techo (Blanco) - Normal hacia abajo (0, -1, 0)
        lista_objetos.append(Quad(Vec3(555,555,555), Vec3(0,0,-555), Vec3(-555,0,0), blanco))
        # Fondo (Blanco) - Normal hacia la cámara (0, 0, -1)
        lista_objetos.append(Quad(Vec3(0,0,555), Vec3(555,0,0), Vec3(0,555,0), blanco))

        # LUZ DE TECHO
        lista_objetos.append(Quad(Vec3(213, 554.9, 227), Vec3(130, 0, 0), Vec3(0, 0, 105), 
                                  Vec3(0,0,0), emission=luz_emision))

        # MODELO 3D
        params_vidrio = {'is_dielectric': True, 'ior': 1.5}
        modelo_triangulos = load_obj(
            "models/bunny.obj", 
            color=Vec3(0.9, 0.9, 0.9), 
            offset=Vec3(278, 0, 278), # Centrado en el piso
            scale=3000.0,             # <--- Subimos de 150 a 3000
            material_params={'is_dielectric': True, 'ior': 1.5} # Vidrio para que sea pro
        )
        lista_objetos.extend(modelo_triangulos)

    world = BVHNode.create(lista_objetos)
    lights = [obj for obj in lista_objetos if obj.emission.length() > 0]
    return world, lights


if __name__ == "__main__":
    render()
