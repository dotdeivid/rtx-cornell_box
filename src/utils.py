import random
from src.vector import Vec3
from src.geometry import Triangle


def random_in_unit_sphere():
    """
    Genera un vector aleatorio dentro de la esfera unitaria usando método de rechazo.

    Returns:
        Vec3: Vector con magnitud < 1, distribuido uniformemente en el volumen
    """
    while True:
        # Generamos un punto en un cubo de -1 a 1
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

        # ¿Está dentro de la esfera? (Método de rechazo)
        if p.length() < 1.0:
            return p


def generar_direccion_aleatoria(normal):
    """
    Genera una dirección unitaria aleatoria en el hemisferio orientado por la normal.

    Args:
        normal (Vec3): Normal de la superficie (debe ser unitaria)

    Returns:
        Vec3: Vector unitario con ángulo < 90° respecto a la normal
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


def load_obj(filename, scale=1.0, offset=None, material=None):
    """
    Carga un archivo .obj y lo convierte en lista de triángulos.

    Soporta vértices (v) y caras (f) con índices opcionales de textura/normal.
    Triangula polígonos de más de 3 lados usando fan desde el primer vértice.

    Args:
        filename (str): Ruta al archivo .obj
        scale (float): Factor de escala uniforme aplicado a los vértices
        offset (Vec3): Traslación aplicada después del escalado. Por defecto (0,0,0)
        material (Material): Material para todos los triángulos. Por defecto: DiffuseMaterial gris

    Returns:
        list[Triangle]: Lista de triángulos del modelo, vacía si hay error de carga
    """
    vertices = []
    triangles = []

    # Defaults
    if offset is None:
        offset = Vec3(0, 0, 0)
    if material is None:
        from src.materials import DiffuseMaterial

        material = DiffuseMaterial(Vec3(0.8, 0.8, 0.8))

    try:
        with open(filename, "r") as f:
            for line in f:
                if line.startswith("v "):
                    # Vértices: v x y z
                    parts = line.split()
                    v = Vec3(float(parts[1]), float(parts[2]), float(parts[3]))
                    vertices.append(v * scale + offset)

                elif line.startswith("f "):
                    # Caras: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
                    # (Solo nos interesan los índices de los vértices)
                    parts = line.split()[1:]
                    indices = []
                    for p in parts:
                        # El primer número antes de la barra es el índice del vértice
                        idx = int(p.split("/")[0])
                        # Los archivos OBJ usan índices que empiezan en 1
                        indices.append(idx - 1 if idx > 0 else len(vertices) + idx)

                    # Creamos el triángulo con los vértices correspondientes
                    # Soporta polígonos de más de 3 lados haciendo un "fan"
                    for i in range(1, len(indices) - 1):
                        triangles.append(
                            Triangle(
                                vertices[indices[0]],
                                vertices[indices[i]],
                                vertices[indices[i + 1]],
                                material,
                            )
                        )
        print(f"Modelo cargado: {len(triangles)} triángulos.")
    except Exception as e:
        print(f"Error al cargar el archivo OBJ: {e}")

    return triangles


def random_in_unit_disk():
    """
    Genera un punto aleatorio dentro de un disco de radio 1 en el plano XY (método de rechazo).

    Usado para simular depth of field en el modelo de cámara thin-lens.

    Returns:
        Vec3: Punto con x,y ∈ [-1,1], z=0 y distancia al centro < 1
    """
    while True:
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), 0)
        if p.length() < 1.0:
            return p
