FROM carlasim/carla:0.9.6 AS builder

FROM byodr/ubuntu_bionic

COPY ./common common/
COPY ./vehicles/carla09 app/
COPY --from=builder /home/carla/PythonAPI/carla/dist/carla-0.9.6-py2.7-linux-x86_64.egg app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN python -m easy_install --no-deps carla-0.9.6-py2.7-linux-x86_64.egg

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENV PYTHONPATH "${PYTHONPATH}:/common"