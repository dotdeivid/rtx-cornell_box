import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere
from src.utils import generar_direccion_aleatoria
import math

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
        # En lugar de un solo reflejo, generamos una dirección aleatoria
        # en la semiesfera (hemisphere) de la normal.
        target = closest_hit.point + generar_direccion_aleatoria(closest_hit.normal)
        new_ray = Ray(closest_hit.point, target - closest_hit.point)
        
        # El color del objeto (0.0 a 1.0) multiplica la luz que viene del rebote
        return color_ray(new_ray, world, depth - 1) * closest_hit.color

    # FONDO NEGRO: Si no choca con nada, no hay luz.
    # Pero cuidado: si todo es negro y no hay una "lámpara", verás todo negro.
    # Vamos a poner una luz arriba para que "algo" brille.
    if ray.direction.y > 0.8: # Una "luz" en el techo del cielo
        return Vec3(1.5, 1.5, 1.5) # Luz blanca intensa
        
    return Vec3(0.05, 0.05, 0.05) # Un gris casi negro para el resto del espacio

def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    samples = 50 # Número de muestras. A mayor número, mayor nitidez
    depth = 3 # Profundidad de rebotes
    
    # Nuestro "Mundo"
    world = [
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(0.8, 0.8, 0.8)), # Piso gris oscuro (80% de reflexión)
        Sphere(Vec3(0, 0, -1), 0.5, Vec3(1.0, 0.2, 0.2)),       # Esfera Roja (R:1, G:0.2, B:0.2)
        Sphere(Vec3(1, 0, -1), 0.5, Vec3(0.2, 1.0, 0.2))        # Esfera Verde (R:0.2, G:1, B:0.2)
    ]
    
    data = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        print(f"Línea {y}/{height}", end="\r")
        for x in range(width):
            col = Vec3(0, 0, 0)
            for _ in range(samples):
                u = (x / width) * 4 - 2
                v = -((y / height) * 2 - 1)
                
                ray = Ray(camera_origin, Vec3(u, v, -1))
                col = col + color_ray(ray, world, depth)
            
            # Promedio y Corrección Gamma (para que no sea tan oscuro)
            pixel_color = col / samples
            ir = int(255.99 * math.sqrt(pixel_color.x))
            ig = int(255.99 * math.sqrt(pixel_color.y))
            ib = int(255.99 * math.sqrt(pixel_color.z))

            data[y, x] = [
                min(ir, 255), 
                min(ig, 255),
                min(ib, 255)
            ]

    img = Image.fromarray(data, 'RGB')
    img.save('output/raytracing_reflejos.png')
    print("¡Ray Tracing recursivo completado!")

if __name__ == "__main__":
    render()