import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere

# Nueva función para detectar el choque más cercano en un "mundo" de objetos
def color_ray(ray, world, depth):
    if depth <= 0:
        return Vec3(0, 0, 0)

    closest_hit = None
    dist_min = float('inf')

    for obj in world:
        rec = obj.hit(ray)
        if rec and rec.t < dist_min:
            dist_min = rec.t
            closest_hit = rec

    if closest_hit:
        # --- CONFIGURACIÓN DE LUZ ---
        light_pos = Vec3(5, 5, 0)
        light_dir = (light_pos - closest_hit.point).normalize()
        
        # --- CÁLCULO DE SOMBRAS (SHADOW RAY) ---
        # Movemos el origen un poco (0.001) para evitar el "Shadow Acne"
        shadow_origin = closest_hit.point + closest_hit.normal * 0.001
        shadow_ray = Ray(shadow_origin, light_dir)
        
        in_shadow = False
        for obj in world:
            if obj.hit(shadow_ray):
                in_shadow = True
                break
        
        # --- ILUMINACIÓN ---
        if in_shadow:
            # Si hay sombra, solo queda una luz ambiente tenue
            intensity = 0.1
        else:
            # Si no hay sombra, calculamos Lambert
            intensity = max(0.1, closest_hit.normal.dot(light_dir))

        # --- REFLEJOS (RAY TRACING RECURSIVO) ---
        reflected_dir = ray.direction - closest_hit.normal * 2 * ray.direction.dot(closest_hit.normal)
        reflected_ray = Ray(closest_hit.point + closest_hit.normal * 0.001, reflected_dir)
        
        # Color = (Reflejo * factor) + (Color propio * intensidad de luz)
        reflection_color = color_ray(reflected_ray, world, depth - 1)
        return reflection_color * 0.4 + closest_hit.color * intensity * 0.6
    
    # Fondo
    t_fondo = 0.5 * (ray.direction.y + 1.0)
    return Vec3(1, 1, 1) * (1.0 - t_fondo) + Vec3(0.5, 0.7, 1.0) * t_fondo

def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    
    # Nuestro "Mundo"
    world = [
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(200, 200, 200)),# Piso grande
        Sphere(Vec3(0, 0.9, -3), 1.0, Vec3(255, 50, 50)),     # Esfera roja flotando
        Sphere(Vec3(2, 0.3, -4), 0.8, Vec3(50, 255, 50))      # Esfera verde al fondo
    ]
    
    data = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            u = (x / width) * 4 - 2
            v = -((y / height) * 2 - 1)
            
            ray = Ray(camera_origin, Vec3(u, v, -1))
            
            # Llamamos a nuestra función recursiva con profundidad de 3 rebotes
            pixel_color = color_ray(ray, world, depth=3)
            
            data[y, x] = [
                min(255, int(pixel_color.x)), 
                min(255, int(pixel_color.y)), 
                min(255, int(pixel_color.z))
            ]

    img = Image.fromarray(data, 'RGB')
    img.save('output/raytracing_reflejos.png')
    print("¡Ray Tracing recursivo completado!")

if __name__ == "__main__":
    render()