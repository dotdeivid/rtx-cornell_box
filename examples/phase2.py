from src.vector import Vec3

def test_math():
    v1 = Vec3(1, 0, 0)
    v2 = Vec3(0, 1, 0)

    # El producto cruz de X e Y debe ser Z
    v3 = v1.cross(v2)
    print(f"Producto cruz (X x Y): {v3}") # Debería ser Vec3(0, 0, 1)

    # El producto punto de vectores perpendiculares debe ser 0
    dot_prod = v1.dot(v2)
    print(f"Producto punto (perpendicular): {dot_prod}")

    # Normalización
    v4 = Vec3(5, 0, 0)
    print(f"Normalizado de (5,0,0): {v4.normalize()}") # Debería ser Vec3(1, 0, 0)

if __name__ == "__main__":
    test_math()