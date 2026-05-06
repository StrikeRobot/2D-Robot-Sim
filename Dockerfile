FROM python:3.12-slim

# ── system: virtual display + VNC stack + pygame SDL runtime ─────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        x11vnc \
        novnc \
        libsdl2-2.0-0 \
        libsdl2-image-2.0-0 \
        libsdl2-ttf-2.0-0 \
        libgl1 \
        fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# ── Python deps (own layer — only rebuilt when pyproject.toml changes) ────
COPY pyproject.toml .
RUN pip install --no-cache-dir \
        numpy pygame matplotlib pyyaml "scikit-image>=0.22"

# ── source ────────────────────────────────────────────────────────────────
COPY . .
RUN pip install --no-cache-dir -e . --no-deps \
 && chmod +x docker/entrypoint.sh

EXPOSE 11100

ENV DISPLAY=:99 \
    PYTHONUNBUFFERED=1

CMD ["docker/entrypoint.sh"]
