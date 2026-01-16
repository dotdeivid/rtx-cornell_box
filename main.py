import numpy as np
from PIL import Image
from src.vector import Vec3
from src.ray import Ray
from src.geometry import Sphere

def render():
    width, height = 400, 200
    camera_origin = Vec3(0, 0, 0)
    sphere = Sphere(Vec3(0, 0, -5), 1.5, Vec3(255, 50, 50)) # Esfera rojiza
    
    # Definimos una fuente de luz (arriba a la derecha)
    light_pos = Vec3(5, 5, 0).normalize()
    
    data = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            u = (x / width) * 4 - 2
            v = -((y / height) * 2 - 1)
            
            ray = Ray(camera_origin, Vec3(u, v, -1))
            rec = sphere.hit(ray)
            
            if rec:
                # CÁLCULO DE LAMBERT:
                # Producto punto entre la normal y la dirección de la luz
                intensity = max(0.1, rec.normal.dot(light_pos)) 
                
                # Aplicamos la intensidad al color original
                pixel_color = rec.color * intensity
                data[y, x] = [int(pixel_color.x), int(pixel_color.y), int(pixel_color.z)]
            else:
                # Fondo degradado
                t_fondo = 0.5 * (ray.direction.y + 1.0)
                color = Vec3(1, 1, 1) * (1.0 - t_fondo) + Vec3(0.5, 0.7, 1.0) * t_fondo
                data[y, x] = [int(color.x*255), int(color.y*255), int(color.z*255)]

    img = Image.fromarray(data, 'RGB')
    img.save('output/esfera_3d.png')
    print("¡Efecto 3D logrado! Revisa output/esfera_3d.png")

if __name__ == "__main__":
    render()