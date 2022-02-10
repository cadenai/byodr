FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

RUN pip3 install "pymodbus==2.3.0"

RUN pip3 install geographiclib

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"
