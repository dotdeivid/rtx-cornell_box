import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere
from src.utils import generar_direccion_aleatoria
import math
import random


def calculate_nee(rec, world):
    """
    Calcula la iluminación directa muestreando las fuentes de luz.
    """
    direct_light = Vec3(0, 0, 0)
    # Identificamos qué objetos emiten luz
    lights = [obj for obj in world if obj.emission.length() > 0]

    for light in lights:
        # 1. Muestreo: Elegimos un punto en la lámpara
        p_light = light.random_point_on_surface()
        l_vector = p_light - rec.point
        dist_sq = l_vector.dot(l_vector)
        dist = math.sqrt(dist_sq)
        l_dir = l_vector.normalize()

        # 2. Shadow Ray: ¿Hay algo entre el objeto y la luz?
        # Offset 0.001 para evitar "shadow acne"
        shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)

        visible = True
        for obj in world:
            # Si algo choca antes de llegar a la luz, está en sombra
            h = obj.hit(shadow_ray)
            if h and h.t < (dist - 0.001):
                visible = False
                break

        if visible:
            # --- CÁLCULO DE ATENUACIÓN FÍSICA ---
            # 1. Coseno en el objeto (Ley de Lambert)
            cos_theta = max(0, rec.normal.dot(l_dir))

            # 2. Coseno en la fuente de luz (Importante para esferas/áreas)
            light_normal = (p_light - light.center) / light.radius
            cos_theta_light = max(0, light_normal.dot(l_dir * -1))

            # 3. PDF y Área
            area_light = 4 * math.pi * (light.radius ** 2)
            pdf = 1.0 / area_light

            # Fórmula final de luz directa
            # (Emisión * ColorObj * Geometría) / PDF
            geometry_term = (cos_theta * cos_theta_light) / dist_sq
            direct_light = direct_light + (light.emission * rec.color * geometry_term / pdf)

    return direct_light


def color_ray(ray, world, depth, is_primary_ray=True):
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
            return closest_hit.emission if is_primary_ray else Vec3(0, 0, 0)

        # A. LUZ DIRECTA (NEE)
        luz_directa = calculate_nee(closest_hit, world)

        # B. LUZ INDIRECTA (Rebote aleatorio)
        target = closest_hit.point + generar_direccion_aleatoria(closest_hit.normal)
        new_ray = Ray(
            closest_hit.point + closest_hit.normal * 0.001, target - closest_hit.point
        )

        # IMPORTANTE: is_primary_ray=False para que los rebotes no vuelvan a sumar la luz
        luz_indirecta = color_ray(new_ray, world, depth - 1, False) * closest_hit.color

        return luz_directa + luz_indirecta

    # Fondo (Cielo)
    if ray.direction.y > 0.8:
        return Vec3(1.5, 1.5, 1.5)
    return Vec3(0.05, 0.05, 0.05)


def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    samples = 100  # Número de muestras. A mayor número, mayor nitidez
    depth = 4  # Profundidad de rebotes

    # Nuestro "Mundo"
    world = [
        Sphere(
            Vec3(0, -100.5, -1), 100, Vec3(0.8, 0.8, 0.8)
        ),  # Piso gris oscuro (80% de reflexión)
        Sphere(Vec3(-0.6, 0, -1.2), 0.5, Vec3(1.0, 0.1, 0.1)),  # ESFERA ROJA (Mate)
        Sphere(Vec3(0.6, 0, -1.2), 0.5, Vec3(0.1, 1.0, 0.1)),  # ESFERA VERDE (Mate)
        # LA LUZ DE ÁREA (Una esfera blanca muy brillante arriba)
        # El color es Vec3(15, 15, 15) para que ilumine la escena
        Sphere(Vec3(0, 3, -1), 1.5, Vec3(0, 0, 0), emission=Vec3(15, 15, 15)),
    ]

    data = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        print(f"Progreso: {int(y/height*100)}%", end="\r")
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
            data[y, x] = [
                min(255, int(255.99 * math.sqrt(pixel_color.x))),
                min(255, int(255.99 * math.sqrt(pixel_color.y))),
                min(255, int(255.99 * math.sqrt(pixel_color.z))),
            ]

    Image.fromarray(data).save("output/nee.png")
    print("\n¡Render finalizado con Luz de Área!")


if __name__ == "__main__":
    render()
