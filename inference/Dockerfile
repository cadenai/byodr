FROM tensorflow/tensorflow:1.15.0-gpu

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-opencv \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

# http://wiki.ros.org/melodic/Installation/Ubuntu


COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

ENTRYPOINT ["/ros_entrypoint.sh"]