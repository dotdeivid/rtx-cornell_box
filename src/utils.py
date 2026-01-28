import random
from src.vector import Vec3
from src.geometry import Triangle

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

def load_obj(filename, color, offset=Vec3(0,0,0), scale=1.0, material_params=None):
    """
    Carga un archivo .obj básico y lo convierte en una lista de Triángulos.
    """
    vertices = []
    triangles = []
    
    # Parámetros de material por defecto
    m_params = material_params if material_params else {
        'is_metal': False, 'fuzz': 0.0, 'is_dielectric': False, 'ior': 1.5
    }

    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    # Vértices: v x y z
                    parts = line.split()
                    v = Vec3(float(parts[1]), float(parts[2]), float(parts[3]))
                    vertices.append(v * scale + offset)
                
                elif line.startswith('f '):
                    # Caras: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                    # (Solo nos interesan los índices de los vértices)
                    parts = line.split()[1:]
                    indices = []
                    for p in parts:
                        # El primer número antes de la barra es el índice del vértice
                        idx = int(p.split('/')[0])
                        # Los archivos OBJ usan índices que empiezan en 1
                        indices.append(idx - 1 if idx > 0 else len(vertices) + idx)
                    
                    # Creamos el triángulo con los vértices correspondientes
                    # Soporta polígonos de más de 3 lados haciendo un "fan"
                    for i in range(1, len(indices) - 1):
                        triangles.append(Triangle(
                            vertices[indices[0]], 
                            vertices[indices[i]], 
                            vertices[indices[i+1]], 
                            color, **m_params
                        ))
        print(f"Modelo cargado: {len(triangles)} triángulos.")
    except Exception as e:
        print(f"Error al cargar el archivo OBJ: {e}")
        
    return triangles

def random_in_unit_disk():
    """Genera un punto aleatorio dentro de un disco de radio 1."""
    while True:
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), 0)
        if p.length() < 1.0:
            return p