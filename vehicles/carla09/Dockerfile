FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN python -m easy_install --no-deps carla-0.9.6-py2.7-linux-x86_64.egg

ARG app_uid

RUN groupadd --gid 1000 --system app \
  && useradd --uid "${app_uid}" --gid 1000 --system --create-home --shell /bin/bash app

USER app

ENTRYPOINT ["/ros_entrypoint.sh"]