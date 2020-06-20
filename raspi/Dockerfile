FROM raspbian/stretch

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python-dev \
    python-pip \
    python-setuptools \
    python-wheel \
    python-gpiozero \
 && rm -rf /var/lib/apt/lists/*

# Included here to avoid the long build time on minor build changes.
RUN pip install "pyzmq >=18.1, <19.0"

COPY ./common common/
COPY ./raspi app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

CMD ["python", "app.py"]