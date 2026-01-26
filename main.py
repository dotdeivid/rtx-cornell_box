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
        if closest_hit.emission.length() > 0:
            return closest_hit.emission if puede_ver_luz else Vec3(0, 0, 0)

        if closest_hit.is_metal:
            # --- LÓGICA DE METAL ---
            # 1. Calculamos la reflexión perfecta
            reflected_direction = ray.direction.normalize().reflect(closest_hit.normal)

            # 2. Aplicamos rugosidad: sumamos un vector aleatorio dentro de una esfera de radio 'fuzz'
            # Usamos la función que ya tienes para direcciones aleatorias
            perturbacion = generar_direccion_aleatoria(closest_hit.normal) * closest_hit.fuzz

            final_direction = (reflected_direction + perturbacion).normalize()

            # Añadimos un pequeño margen (0.001) para evitar que el rayo choque con la misma esfera
            scattered_ray = Ray(closest_hit.point + closest_hit.normal * 0.001, final_direction)

            # Verificamos que el rayo no se haya metido dentro de la esfera por la rugosidad
            if final_direction.dot(closest_hit.normal) > 0:
                # ¡IMPORTANTE!: puede_ver_luz = True porque es un reflejo especular
                return color_ray(scattered_ray, world, depth - 1, True) * closest_hit.color
            else:
                return Vec3(0, 0, 0)    
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
    for x in range(width):
        col = Vec3(0, 0, 0)
        for _ in range(samples):
            # Antialiasing: lanzamos rayos con un pequeño offset aleatorio
            u = ((x + random.random()) / width) * 4 - 2
            v = -(((y + random.random()) / height) * 2 - 1)

            ray = Ray(camera_origin, Vec3(u, v, -1))
            col = col + color_ray(ray, world, depth)

        # Promediamos, aplicamos corrección gamma para que no sea tan oscuro (sqrt) y escalamos a 255
        pixel_color = col / samples
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
    depth = 4  # Profundidad de rebotes

    # Nuestro "Mundo"
    world = [
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(0.8, 0.8, 0.8)), # Piso gris oscuro (80% de reflexión)
        Sphere(Vec3(-0.6, 0, -1.2), 0.5, Vec3(1.0, 1.0, 1.0), is_metal=True, fuzz=0.2), # Esfera de Metal roja (is_metal=True)
        Sphere(Vec3(0.6, 0, -1.2), 0.5, Vec3(0.1, 1.0, 0.1)), # Esfera Verde Mate
        # LA LUZ DE ÁREA (Una esfera blanca muy brillante arriba)
        # El color es Vec3(15, 15, 15) para que ilumine la escena
        Sphere(Vec3(0, 3, -1), 1.5, Vec3(0, 0, 0), emission=Vec3(15, 15, 15)), # Luz
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

    Image.fromarray(data).save("output/fuzziness.png")
    print("\n¡Render finalizado con Luz de Área!")


if __name__ == "__main__":
    render()
