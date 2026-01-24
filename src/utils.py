import random
from src.vector import Vec3

def obtener_punto_luz_aleatorio(centro_luz, radio_luz):
    """
    Genera un punto aleatorio dentro de la esfera de luz 
    para simular una bombilla real.
    """
    # Aproximación simple: movemos el centro un poco al azar en X, Y, Z
    offset = Vec3(
        (random.random() - 0.5) * 2,
        (random.random() - 0.5) * 2,
        (random.random() - 0.5) * 2
    ) * radio_luz
    return centro_luz + offset

def random_in_unit_sphere():
    """Genera un vector aleatorio dentro de una esfera de radio 1."""
    while True:
        # Generamos un punto en un cubo de -1 a 1
        p = Vec3(
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        )

        # ¿Está dentro de la esfera? (Método de rechazo)
        if p.length() < 1.0:
            return p

def generar_direccion_aleatoria(normal):
    """
    Genera una dirección unitaria aleatoria en la semiesfera 
    definida por la normal de la superficie.
    """
    # 1. Obtenemos un punto aleatorio normalizado (está en la superficie de la esfera)
    random_dir = random_in_unit_sphere().normalize()
    
    # 2. Verificamos si está en el mismo hemisferio que la normal
    # Si el producto punto es > 0, el ángulo es < 90° (está "hacia afuera")
    if random_dir.dot(normal) > 0.0:
        return random_dir
    else:
        # Si apunta hacia adentro, lo invertimos
        return random_dir * -1