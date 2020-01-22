FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-opencv \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade setuptools wheel

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN python -m easy_install --no-deps carla-0.9.6-py2.7-linux-x86_64.egg

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENTRYPOINT ["/ros_entrypoint.sh"]