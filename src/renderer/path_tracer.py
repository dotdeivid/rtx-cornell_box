"""Path tracer core - rendering engine with NEE."""

import math
from typing import List
from src.vector import Vec3
from src.ray import Ray
from src.geometry import BVHNode, HitRecord
from src.materials.diffuse import DiffuseMaterial


def calculate_nee(rec: HitRecord, world: BVHNode, lights: List, albedo: Vec3) -> Vec3:
    """
    Next Event Estimation - iluminación directa.

    Args:
        rec: HitRecord del punto golpeado
        world: Geometría de la escena
        lights: Lista de objetos emisores
        albedo: Color del material (atenuación devuelta por scatter())

    Returns:
        Color de iluminación directa
    """
    direct_light = Vec3(0, 0, 0)

    for light in lights:
        # Muestrear dirección hacia la luz
        l_dir, solid_angle = light.sample_solid_angle(rec.point)
        cos_theta_surface = max(0, l_dir.dot(rec.normal))

        if cos_theta_surface > 0:
            # Shadow ray
            shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)
            shadow_hit = world.hit(shadow_ray, 0.001, float("inf"))

            # Si golpea la luz (no hay oclusión)
            if shadow_hit and shadow_hit.obj_ref == light:
                brdf = albedo / math.pi
                direct_light += (
                    light.material.emitted() * brdf * cos_theta_surface * solid_angle
                )

    return direct_light


def color_ray(
    ray: Ray,
    world: BVHNode,
    lights: List,
    depth: int,
    max_depth: int,
    puede_ver_luz: bool = True,
) -> Vec3:
    """
    Path tracing recursivo con NEE usando Material ABC.

    Args:
        ray: Rayo a trazar
        world: Geometría de la escena
        lights: Fuentes de luz
        depth: Profundidad actual
        max_depth: Profundidad máxima
        puede_ver_luz: Si puede ver emisión directa

    Returns:
        Color del rayo
    """
    if depth <= 0:
        return Vec3(0, 0, 0)

    hit = world.hit(ray, 0.001, float("inf"))

    if hit:
        # Emisión directa del material
        emission = hit.material.emitted() if hit.material else Vec3(0, 0, 0)
        if emission.length() > 0:
            return emission if puede_ver_luz else Vec3(0, 0, 0)

        # Usar material.scatter() para determinar ray siguiente
        scattered, attenuation, scattered_ray = hit.material.scatter(ray, hit)

        if not scattered:
            # Material absorbió el rayo
            return emission

        # Para materiales difusos, calcular NEE
        if isinstance(hit.material, DiffuseMaterial):
            nee_contribution = calculate_nee(hit, world, lights, attenuation)
            indirect = color_ray(
                scattered_ray, world, lights, depth - 1, max_depth, False
            )

            # BRDF Lambertiano
            # Corrección: El modelo de Lambert ya incluye la probabilidad en el scatter.
            # No multiplicamos por cos_theta ni 2pi porque se cancela con la PDF.
            return nee_contribution + attenuation * indirect
        else:
            # Materiales especulares (metal, dielectric) - no NEE
            # Corrección: Deben poder ver la luz (True) porque no usan NEE.
            indirect = color_ray(
                scattered_ray, world, lights, depth - 1, max_depth, True
            )
            return attenuation * indirect

    # Fondo/cielo
    if ray.direction.y > 0.8:
        return Vec3(1.5, 1.5, 1.5)
    return Vec3(0.05, 0.05, 0.05)
