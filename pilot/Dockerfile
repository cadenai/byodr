# docker build -t caden/teleop .
FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . app/

RUN pip install --no-cache-dir -r requirements.txt

ENV PILOT_HOME /app

ENTRYPOINT ["python", "pilot.py", "--config=9100"]



