import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere

def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    sphere = Sphere(Vec3(0, 0, -5), 1.0, Vec3(255, 0, 0)) # Esfera roja
    
    # Creamos el canvas de la imagen
    data = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            # Mapeamos coordenadas de pantalla a espacio 3D (-2 a 2 aprox)
            u = (x / width) * 4 - 2
            v = -((y / height) * 2 - 1)
            
            direction = Vec3(u, v, -1) # Apuntamos hacia adelante
            ray = Ray(camera_origin, direction)
            
            if sphere.hit(ray):
                data[y, x] = [255, 0, 0] # Pintar de rojo si hay hit
            else:
                # Fondo: un degradado azul/blanco simple
                t_fondo = 0.5 * (ray.direction.y + 1.0)
                color = Vec3(1, 1, 1) * (1.0 - t_fondo) + Vec3(0.5, 0.7, 1.0) * t_fondo
                data[y, x] = [int(color.x*255), int(color.y*255), int(color.z*255)]

    img = Image.fromarray(data, 'RGB')
    img.save('output/primera_esfera.png')
    print("Render finalizado. ¡Mira output/primera_esfera.png!")

if __name__ == "__main__":
    render()