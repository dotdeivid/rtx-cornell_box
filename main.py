import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere
from src.utils import generar_direccion_aleatoria
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
    r_out_parallel = n * -math.sqrt(abs(1.0 - r_out_perp.length()**2))
    return r_out_perp + r_out_parallel

def reflectance(cosine, ref_idx):
    """Aproximación de Schlick para determinar la probabilidad de reflexión."""
    r0 = (1 - ref_idx) / (1 + ref_idx)
    r0 = r0 * r0
    return r0 + (1 - r0) * math.pow((1 - cosine), 5)

def calculate_nee(rec, world):
    direct_light = Vec3(0, 0, 0)
    lights = [obj for obj in world if obj.emission.length() > 0]
    
    for light in lights:
        # 1. Obtenemos dirección y el ángulo sólido que ocupa la luz
        l_dir, solid_angle = light.sample_solid_angle(rec.point)
        
        # 2. Shadow Ray: Verificamos visibilidad
        shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)
        
        # Verificamos si golpeamos la luz directamente sin obstáculos
        # (Optimizamos: solo nos importa si el primer hit es la luz)
        hit_light = None
        min_t = float('inf')
        for obj in world:
            h = obj.hit(shadow_ray)
            if h and h.t < min_t:
                min_t = h.t
                hit_light = obj
        
        if hit_light == light:
            # 3. Cálculo de Iluminancia
            # cos_theta: Cuánta luz recibe la superficie según su inclinación
            cos_theta = max(0, rec.normal.dot(l_dir))
            
            # Con Muestreo de Ángulo Sólido, la fórmula es:
            # Luz = Emisión * Color * cos(theta) * (Ángulo Sólido / PI)
            # El PI viene de la normalización del material difuso (Lambert)
            direct_light = direct_light + (light.emission * rec.color * (cos_theta * solid_angle / math.pi))
            
    return direct_light

def color_ray(ray, world, depth, puede_ver_luz=True):
    if depth <= 0:
        return Vec3(0, 0, 0)

    closest_hit = None
    dist_min = float("inf")

    for obj in world:
        rec = obj.hit(ray)
        if rec and rec.t < dist_min:
            dist_min = rec.t
            closest_hit = rec

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
            perturbacion = generar_direccion_aleatoria(closest_hit.normal) * closest_hit.fuzz

            final_direction = (reflected_direction + perturbacion).normalize()


            # Verificamos que el rayo no se haya metido dentro de la esfera por la rugosidad
            if final_direction.dot(closest_hit.normal) > 0:
                # Añadimos un pequeño margen (0.001) para evitar que el rayo choque con la misma esfera
                scattered_ray = Ray(closest_hit.point + closest_hit.normal * 0.001, final_direction)
                # ¡IMPORTANTE!: puede_ver_luz = True porque es un reflejo especular
                return color_ray(scattered_ray, world, depth - 1, True) * closest_hit.color
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
            return color_ray(scattered_ray, world, depth - 1, True) * closest_hit.color 
        else:
            # A. LUZ DIRECTA (NEE)
            luz_directa = calculate_nee(closest_hit, world)

            # B. LUZ INDIRECTA (Rebote aleatorio)
            target = closest_hit.point + generar_direccion_aleatoria(closest_hit.normal)
            new_ray = Ray(
                closest_hit.point + closest_hit.normal * 0.001, target - closest_hit.point
            )

            # IMPORTANTE: puede_ver_luz=False para que los rebotes no vuelvan a sumar la luz
            luz_indirecta = color_ray(new_ray, world, depth - 1, False) * closest_hit.color

            return luz_directa + luz_indirecta

    # Fondo (Cielo)
    if ray.direction.y > 0.8:
        return Vec3(1.5, 1.5, 1.5)
    return Vec3(0.05, 0.05, 0.05)

def render_row(y, width, height, samples, depth, world, camera_origin):
    """
    Función que procesa una sola fila de la imagen. 
    Esta función es la que se distribuirá entre los núcleos.
    """
    row_pixels = []
    # Calculamos la raíz de las muestras para crear una cuadrícula (ej: sqrt(100) = 10)
    s_side = int(math.sqrt(samples))


    for x in range(width):
        col = Vec3(0, 0, 0)
        
        # Bucle de Antialiasing Estratificado (Cuadrícula)
        for i in range(s_side):
            for j in range(s_side):
                # Dividimos el píxel en sub-celdas y lanzamos un rayo en cada una
                u_offset = (i + random.random()) / s_side
                v_offset = (j + random.random()) / s_side
                
                u = ((x + u_offset) / width) * 4 - 2
                v = -(((y + v_offset) / height) * 2 - 1)

                ray = Ray(camera_origin, Vec3(u, v, -1))
                col = col + color_ray(ray, world, depth)

        # Promediamos por el total real de muestras (s_side * s_side)
        pixel_color = col / (s_side * s_side)

        # Corrección gamma y escalado a 255
        r = min(255, int(255.99 * math.sqrt(pixel_color.x)))
        g = min(255, int(255.99 * math.sqrt(pixel_color.y)))
        b = min(255, int(255.99 * math.sqrt(pixel_color.z)))
        row_pixels.append([r, g, b])

    return row_pixels



def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    samples = 100  # Número de muestras. A mayor número, mayor nitidez
    depth = 8 # Profundidad de rebotes

    # Nuestro "Mundo"
    world = [
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(0.8, 0.8, 0.8)),
        # Metal cromado a la izquierda
        Sphere(Vec3(-0.6, 0, -1.2), 0.5, Vec3(1.0, 1.0, 1.0), is_metal=True, fuzz=0.0),
        # VIDRIO a la derecha (IOR 1.5)
        Sphere(Vec3(0.6, 0, -1.2), 0.5, Vec3(1.0, 1.0, 1.0), is_dielectric=True, ior=1.5),
        Sphere(Vec3(0, 3, -1), 1.5, Vec3(0, 0, 0), emission=Vec3(15, 15, 15)),
    ]

    if USE_PARALLEL:
        num_cores = multiprocessing.cpu_count()
        print(f"Iniciando render paralelo con {num_cores} núcleos...")
        
        # Preparamos la función con los argumentos constantes
        worker_func = partial(render_row, width=width, height=height, samples=samples, 
                              depth=depth, world=world, camera_origin=camera_origin)
        
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

            row_data = render_row(y, width, height, samples, depth, world, camera_origin)
            data[y] = row_data

    Image.fromarray(data).save("output/antialiasing.png")
    print("\n¡Render finalizado con Antialiasing!")


if __name__ == "__main__":
    render()

