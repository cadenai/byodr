FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENTRYPOINT ["/ros_entrypoint.sh"]