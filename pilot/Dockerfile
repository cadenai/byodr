FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

#RUN pip install --upgrade setuptools wheel

COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

ENTRYPOINT ["/ros_entrypoint.sh"]