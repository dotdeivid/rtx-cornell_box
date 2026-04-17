# `src/vector.py` — Clase Vec3

Estructura de datos fundamental del proyecto. Representa vectores 3D y se usa para posiciones, direcciones, colores y cualquier cantidad tridimensional.

---

## Clase `Vec3`

Almacena internamente un `numpy.ndarray` de 3 elementos `float64`. Todas las operaciones retornan un **nuevo** `Vec3` sin modificar el original (inmutabilidad).

### Constructor

```python
Vec3(x: float, y: float, z: float)
```

Convierte los valores a `float64`. Acepta enteros o flotantes.

```python
Vec3(1, 0, 0)        # Dirección +X
Vec3(0.8, 0.2, 0.2)  # Color rojizo
Vec3(278, 278, -800) # Posición en escena
```

### Propiedades de solo lectura

| Propiedad | Descripción |
|---|---|
| `.x` | Componente X (`components[0]`) |
| `.y` | Componente Y (`components[1]`) |
| `.z` | Componente Z (`components[2]`) |

---

## Operadores aritméticos

### `__add__(other: Vec3) → Vec3`
Suma componente a componente: `(a+b, c+d, e+f)`.

Usos: trasladar puntos, combinar colores, acumular muestras.

### `__sub__(other: Vec3) → Vec3`
Resta componente a componente.

Usos: obtener vector dirección (`target - origin`), diferencia de posiciones.

### `__mul__(other: Vec3 | float) → Vec3`
**Dos modos:**
- `Vec3 * float` → escalar: multiplica cada componente por el número.
- `Vec3 * Vec3` → Hadamard (elemento a elemento): `(a·d, b·e, c·f)`.

El producto Hadamard se usa para atenuar colores: `light_color * surface_albedo`.

### `__rmul__(scalar: float) → Vec3`
Permite `2 * v` además de `v * 2`.

### `__truediv__(scalar: float) → Vec3`
División por escalar: `(x/k, y/k, z/k)`. No soporta división Vec3 / Vec3.

### `__neg__() → Vec3`
Negación: `(-x, -y, -z)`. Invierte la dirección.

### `__iadd__(other: Vec3) → Vec3`
Suma en sitio: `v += other`. Usado en el renderer para acumular samples.

---

## Álgebra lineal

### `dot(other: Vec3) → float`
Producto punto: `x·ox + y·oy + z·oz`.

Equivale a `|v1| · |v2| · cos(θ)` donde θ es el ángulo entre los vectores. Usos:
- Comprobar si dos vectores apuntan en la misma dirección (`> 0`) o contraria (`< 0`)
- Calcular cos(θ) si ambos vectores son unitarios
- Test de hemisferio para normales

### `cross(other: Vec3) → Vec3`
Producto cruzado: vector perpendicular a ambos.

```
resultado = (y·oz - z·oy,  z·ox - x·oz,  x·oy - y·ox)
```

Usos: calcular normales de superficies (triángulos, quads), construir bases ortonormales para la cámara.

### `length() → float`
Magnitud euclidiana: `√(x² + y² + z²)`.

### `length_squared() → float`
Cuadrado de la magnitud: `x² + y² + z²`. Más barato que `length()` (evita la raíz cuadrada). Útil cuando solo se necesita comparar magnitudes.

### `normalize() → Vec3`
Retorna el vector unitario: `v / v.length()`. Si el vector es cero retorna Vec3(0,0,0).

Esencial para: direcciones de rayos, normales de superficie, ejes de cámara.

---

## Métodos de reflexión y refracción

### `reflect(normal: Vec3) → Vec3`
Calcula la dirección reflejada respecto a una normal:

```
R = I - 2·(I·N)·N
```

`self` es el vector incidente `I`, `normal` es `N`. El resultado es la dirección de rebote especular (espejos, metales).

### `refract(normal: Vec3, etai_over_etat: float) → Vec3`
Calcula la dirección refractada usando la Ley de Snell vectorial.

`etai_over_etat` es el cociente de índices de refracción η₁/η₂. Descompone el vector en componente perpendicular y paralela a la normal para aplicar Snell.

---

## Representación

### `__repr__() → str`
Retorna `Vec3(x, y, z)` con los valores reales.

### `__eq__(other) → bool`
Igualdad aproximada con tolerancia `1e-9` por componente (evita problemas de punto flotante).
