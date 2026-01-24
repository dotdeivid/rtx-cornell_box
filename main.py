import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere
from src.utils import generar_direccion_aleatoria
import math
import random

# Nueva función para detectar el choque más cercano en un "mundo" de objetos
def color_ray(ray, world, depth):
    # Si excedemos los rebotes, no hay más luz
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
        # 1. Si chocamos con una LUZ, devolvemos su brillo
        if closest_hit.emission.length() > 0:
            return closest_hit.emission

        # 2. Si chocamos con un objeto normal, rebotamos (Monte Carlo)
        # En lugar de un solo reflejo, generamos una dirección aleatoria
        # en la semiesfera (hemisphere) de la normal.
        target = closest_hit.point + generar_direccion_aleatoria(closest_hit.normal)
        new_ray = Ray(closest_hit.point, target - closest_hit.point)
        
        # El color del objeto (0.0 a 1.0) multiplica la luz que viene del rebote
        # El color resultante es: (Luz del rebote) * (Color del objeto)
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
    samples = 100 # Número de muestras. A mayor número, mayor nitidez
    depth = 4 # Profundidad de rebotes
    
    # Nuestro "Mundo"
    world = [
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(0.8, 0.8, 0.8)), # Piso gris oscuro (80% de reflexión)
        Sphere(Vec3(-0.6, 0, -1.2), 0.5, Vec3(1.0, 0.1, 0.1)),       # ESFERA ROJA (Mate)
        Sphere(Vec3(0.6, 0, -1.2), 0.5, Vec3(0.1, 1.0, 0.1)),       # ESFERA VERDE (Mate)
        
        # LA LUZ DE ÁREA (Una esfera blanca muy brillante arriba)
        # El color es Vec3(15, 15, 15) para que ilumine la escena
        Sphere(Vec3(0, 3, -1), 1.5, Vec3(0,0,0), emission=Vec3(15, 15, 15))
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
                min(255, int(255.99 * math.sqrt(pixel_color.z)))
            ]

    Image.fromarray(data).save('output/soft_shadows_montecarlo.png')
    print("\n¡Render finalizado con Luz de Área!")

if __name__ == "__main__":
    render()