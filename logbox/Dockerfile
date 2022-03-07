FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

RUN pip3 install "pymongo==4.0.1"

COPY ./common common/
COPY ./logbox app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"
