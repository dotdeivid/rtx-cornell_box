# =============================================================================
# Dockerfile - RTX Cornell Box Path Tracer (Versión Simplificada)
# =============================================================================
# Uso:
#   docker build -t rtx-cornell-box .
#   docker run -v $(pwd)/output:/app/output rtx-cornell-box
# =============================================================================

# Usamos una imagen base ligera de Python 3.11
FROM python:3.11-slim-bookworm

# Metadatos
LABEL maintainer="Sandoval, Carlos David"
LABEL description="Path Tracer Cornell Box - Versión Simplificada"

# Configuración de entorno
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONOPTIMIZE=1 \
    # Variables de renderizado por defecto
    RTX_WIDTH=400 \
    RTX_HEIGHT=400 \
    RTX_SAMPLES=400 \
    RTX_DEPTH=8

# Directorio de trabajo
WORKDIR /app

# Instalar dependencias del sistema mínimas
# (Limpiamos cache en la misma capa para mantener la imagen pequeña)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libjpeg62-turbo \
    libpng16-16 \
    && rm -rf /var/lib/apt/lists/*

# Copiar requirements e instalar dependencias Python
# Lo hacemos antes de copiar el código para aprovechar el cache de Docker
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Crear usuario no-root por seguridad
RUN groupadd -g 1000 appgroup && \
    useradd -u 1000 -g appgroup -m appuser && \
    # Crear directorio de output y dar permisos
    mkdir -p /app/output && \
    chown -R appuser:appgroup /app

# Copiar el código fuente
COPY --chown=appuser:appgroup . .

# Usar el usuario no-root
USER appuser

# Volumen para persistir las imágenes generadas
VOLUME ["/app/output"]

# Comando por defecto
CMD ["python", "main.py"]
