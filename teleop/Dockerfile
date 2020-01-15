FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python-opencv \
    python-pip \
    ros-melodic-cv-bridge \
    ros-melodic-vision-opencv \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . app/

RUN pip install --no-cache-dir -r requirements.txt

ENV TELEOP_HOME /app

EXPOSE 9100

ENTRYPOINT ["python", "teleop.py", "--port=9100"]



