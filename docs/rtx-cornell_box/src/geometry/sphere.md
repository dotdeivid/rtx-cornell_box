# `src/geometry/sphere.py` — Clase `Sphere`

Primitiva geométrica esfera. La más simple matemáticamente y la única con un método de muestreo de ángulo sólido propio (usado si se colocan esferas como fuentes de luz).

---

## Clase `Sphere`

### Constructor

```python
Sphere(center: Vec3, radius: float, material: Material)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `center` | `Vec3` | Centro de la esfera en el espacio 3D |
| `radius` | `float` | Radio de la esfera |
| `material` | `Material` | Material que define el comportamiento óptico |

---

## Métodos

### `hit(ray, t_min, t_max) → HitRecord | None`

Intersección rayo-esfera mediante la ecuación cuadrática.

**Derivación matemática:**

Una esfera es el conjunto de puntos P con `||P - C||² = r²`. Un rayo es `P(t) = O + t·D`. Sustituyendo:

```
||O + t·D - C||² = r²

Sea oc = O - C:
  t²(D·D) + 2t(D·oc) + (oc·oc - r²) = 0

a = D·D  (= 1 porque D está normalizado)
b = 2·(D·oc)
c = oc·oc - r²

Δ = b² - 4ac
```

- `Δ < 0` → sin intersección, retorna `None`
- `Δ ≥ 0` → dos raíces `(-b ± √Δ) / 2a`

Se elige la raíz menor (entrada) si está en `[t_min, t_max]`. Si no, se prueba la raíz mayor (salida). Si ninguna es válida, retorna `None`.

**Normal:**
```python
normal = (point - self.center) / self.radius
```
Este vector es unitario porque `||point - center|| = radius` por definición de esfera.

**Retorna:** `HitRecord(t, point, normal, material, obj_ref=self)` o `None`.

---

### `random_point_on_surface() → Vec3`

Genera un punto aleatorio con distribución **uniforme en el área** de la superficie.

**Algoritmo:**
```python
theta = 2π · random()           # Ángulo azimutal
phi   = arccos(2·random() - 1)  # Ángulo polar (distribución uniforme en área)

x = sin(phi)·cos(theta)
y = sin(phi)·sin(theta)
z = cos(phi)

point = center + direction * radius
```

La distribución `phi = arccos(2u - 1)` compensa que las zonas polares tienen menos área que el ecuador, garantizando uniformidad. Un simple `phi = u·π` sobremuestrearía los polos.

**Uso:** muestreo de esferas luminosas (si se usara una esfera como fuente de luz).

---

### `sample_solid_angle(hit_point: Vec3) → tuple[Vec3, float]`

Muestreo por ángulo sólido (importance sampling) para esferas luminosas.

En lugar de muestrear puntos en toda la superficie de la esfera, genera direcciones dentro del **cono** que la esfera subtiende vista desde `hit_point`. Esto concentra las muestras donde realmente pueden contribuir luz.

**Cálculo del cono:**
```
sin²(θ_max) = r² / d²          (d = distancia al centro)
cos(θ_max)  = √(1 - sin²(θ_max))
```

**Muestreo dentro del cono:**
```
phi      = 2π · r1
cos(θ)   = 1 - r2·(1 - cos(θ_max))    (uniforme entre 1 y cos(θ_max))
sin(θ)   = √(1 - cos²(θ))
```

**Transformación a espacio mundo:**
Se construye una base ortonormal local (x_axis, y_axis, z_axis) con z_axis apuntando al centro de la esfera, y se transforma la dirección local a coordenadas mundo.

**Ángulo sólido:**
```
Ω = 2π · (1 - cos(θ_max))
```

**Retorna:** `(dirección_normalizada: Vec3, ángulo_sólido: float)`

---

### `bounding_box() → AABB`

Caja envolvente trivial para la esfera:

```
min = center - Vec3(radius, radius, radius)
max = center + Vec3(radius, radius, radius)
```

El cubo resultante toca la esfera exactamente en 6 puntos (los extremos de cada eje).
