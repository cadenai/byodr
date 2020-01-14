# docker build -t byodr/carla .
FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . app/

RUN pip install --no-cache-dir -r requirements.txt
# pip install -r https://raw.githubusercontent.com/carla-simulator/carla/0.9.6/PythonClient/requirements.txt
# pip install git+https://github.com/carla-simulator/carla.git@0.9.6#"egg=carla_client&subdirectory=PythonClient"

#EXPOSE 9100

