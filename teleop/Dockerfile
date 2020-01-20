FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-opencv \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 9100

ENTRYPOINT ["/ros_entrypoint.sh"]