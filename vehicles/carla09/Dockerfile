FROM carlasim/carla:0.9.9 AS builder

FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

RUN pip3 install -r https://raw.githubusercontent.com/carla-simulator/carla/0.9.9/PythonAPI/carla/requirements.txt

COPY ./common common/
COPY ./vehicles/carla09 app/
COPY --from=builder /home/carla/PythonAPI/ PythonAPI/
WORKDIR /app

RUN python3 -m easy_install --no-deps /PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg

RUN pip3 install geographiclib

RUN pip3 install "tornado==4.5.3"

ENV PYTHONPATH "${PYTHONPATH}:/common"
