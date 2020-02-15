FROM byodr/ubuntu_bionic

RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
 && rm -rf /var/lib/apt/lists/*

COPY ./common common/
COPY ./teleop app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

RUN mkdir /byodr && chown -R app:app /byodr

VOLUME /byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

EXPOSE 9100