"""Path tracer core - motor de rendering Monte Carlo con NEE y Russian Roulette."""

import math
import random
from typing import Protocol, Sequence, Tuple, runtime_checkable

from src.vector import Vec3
from src.ray import Ray
from src.geometry.base import Geometry
from src.geometry.hit_record import HitRecord
from src.materials.base import Material


@runtime_checkable
class AreaLight(Protocol):
    """Interfaz esperada para objetos emisores muestreados por NEE."""

    material: Material

    def sample_solid_angle(self, hit_point: Vec3) -> Tuple[Vec3, float]: ...


def calculate_nee(
    rec: HitRecord,
    world: Geometry,
    lights: Sequence[AreaLight],
    albedo: Vec3,
) -> Vec3:
    """
    Next Event Estimation - iluminación directa desde fuentes de luz de área.

    Args:
        rec: HitRecord del punto golpeado
        world: Geometría de la escena (BVH o primitiva)
        lights: Lista de objetos emisores
        albedo: Color del material (atenuación devuelta por ``scatter()``)

    Returns:
        Contribución de iluminación directa
    """
    direct_light = Vec3(0.0, 0.0, 0.0)
    shadow_origin = rec.point + rec.normal * 1e-4

    for light in lights:
        l_dir, solid_angle = light.sample_solid_angle(rec.point)
        if solid_angle <= 0.0:
            continue

        cos_theta_surface = l_dir.dot(rec.normal)
        if cos_theta_surface > 0.0:
            shadow_ray = Ray(shadow_origin, l_dir)
            shadow_hit = world.hit(shadow_ray, 1e-4, float("inf"))

            if shadow_hit is not None and shadow_hit.obj_ref is light:
                brdf = albedo / math.pi
                direct_light = direct_light + (
                    light.material.emitted()
                    * brdf
                    * (cos_theta_surface * solid_angle)
                )

    return direct_light


def color_ray(
    ray: Ray,
    world: Geometry,
    lights: Sequence[AreaLight],
    depth: int,
    max_depth: int,
    can_see_light: bool = True,
) -> Vec3:
    """
    Path tracing recursivo con NEE y Russian Roulette sin sesgo.

    Args:
        ray: Rayo a trazar
        world: Geometría de la escena
        lights: Fuentes de luz muestreadas por NEE
        depth: Profundidad restante
        max_depth: Profundidad inicial (usada para activar Russian Roulette)
        can_see_light: Si puede sumar emisión directa de luces muestreadas por NEE

    Returns:
        Radiancia estimada para el rayo
    """
    if depth <= 0:
        return Vec3(0.0, 0.0, 0.0)

    hit = world.hit(ray, 0.001, float("inf"))
    if hit is None or hit.material is None:
        # Cornell Box cerrada - sin cielo/fondo
        return Vec3(0.0, 0.0, 0.0)

    # Emisión directa del material
    emission = hit.material.emitted()
    if emission.length_squared() > 0.0:
        if not hit.front_face:
            return Vec3(0.0, 0.0, 0.0)
        # Solo omitir si este emisor ya está siendo muestreado por NEE (en lights)
        is_sampled_light = any(hit.obj_ref is light for light in lights)
        if can_see_light or not is_sampled_light:
            return emission
        return Vec3(0.0, 0.0, 0.0)

    scattered, attenuation, scattered_ray = hit.material.scatter(ray, hit)
    if not scattered or attenuation is None or scattered_ray is None:
        return Vec3(0.0, 0.0, 0.0)

    uses_nee = hit.material.uses_nee
    nee_contribution = (
        calculate_nee(hit, world, lights, attenuation)
        if uses_nee
        else Vec3(0.0, 0.0, 0.0)
    )

    # Russian Roulette a partir del 3er rebote para evitar trabajo en caminos de baja energía
    bounces = max_depth - depth
    rr_scale = 1.0
    if bounces >= 3:
        survival_prob = max(
            0.05, min(0.95, max(attenuation.x, attenuation.y, attenuation.z))
        )
        if random.random() > survival_prob:
            return nee_contribution
        rr_scale = 1.0 / survival_prob

    # Materiales que usan NEE ocultan luces muestreadas en el rebote (can_see_light=False)
    indirect = color_ray(
        scattered_ray,
        world,
        lights,
        depth - 1,
        max_depth,
        not uses_nee,
    )

    return nee_contribution + attenuation * indirect * rr_scale
