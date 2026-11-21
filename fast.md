# Guía rápida de ejecución

Desde la raíz del proyecto, activando el entorno virtual:

```bash
source venv/bin/activate
```

## 1. Renderizar una imagen

- **Vista previa rápida** (recomendado para probar):
  ```bash
  python main.py --preset PREVIEW
  ```
- **Ver todos los presets disponibles**:
  ```bash
  python main.py --list-presets
  ```
- **Ejecutar con el preset por defecto** (`TINY_PERFECT` en `main.py`):
  ```bash
  python main.py
  ```
  > Nota: `TINY_PERFECT` usa `2000` samples y `max_depth=16`, por lo que tarda varios minutos.

## 2. Ejemplos con opciones de CLI

```bash
# Renderizar la escena de cajas o el Stanford Bunny
python main.py --preset BOXES
python main.py --preset BUNNY

# Personalizar resolución, samples, semilla y salida sin editar código
python main.py --preset PREVIEW --scene boxes --width 300 --height 300 --samples 100 --seed 42 --output output/mi_render.png

# Forzar ejecución en un solo núcleo (sin multiprocessing)
python main.py --preset PREVIEW --sequential
```

La imagen generada se guarda en la ruta indicada (`output/` por defecto).

## 3. Correr tests y verificación de tipos

```bash
pytest -v
mypy src main.py
```

> Si no activás el `venv`, podés ejecutar directamente `./venv/bin/python main.py ...`, `./venv/bin/pytest -v` y `./venv/bin/mypy src main.py`.
