FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python-opencv \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . app/

RUN pip install --no-cache-dir -r requirements.txt

# USER
RUN python -m easy_install --no-deps carla-0.9.6-py2.7-linux-x86_64.egg
