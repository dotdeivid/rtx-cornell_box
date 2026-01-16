import numpy as np
from PIL import Image
import os

def crear_test():
    # Crear una carpeta para las imágenes si no existe
    if not os.path.exists('output'):
        os.makedirs('output')

    # Crear una imagen negra de 100x100 (RGB)
    # (100, 100, 3) matriz tridimensional de 100 filas por 100 columnas por 3 canales (RGB)
    # Almacena valores enteros de 0 a 255 (uint8)
    # Tipicamente, utilizado para crear imagenes
    data = np.zeros((100, 100, 3), dtype=np.uint8)
    
    # Dibujar un cuadrado rojo en el centro (Test de NumPy)
    # Se coloca en la fila 40 a 60 y la columna 40 a 60
    # En la interseccion de estas filas y columnas se coloca un cuadrado rojo
    data[40:60, 40:60] = [255, 0, 0]
    
    img = Image.fromarray(data, 'RGB')
    img.save('output/test_entorno.png')
    print("¡Entorno listo! Revisa la carpeta output/test_entorno.png")

if __name__ == "__main__":
    crear_test()