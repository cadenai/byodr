FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

RUN pip3 install "tornado==4.5.3"

COPY ./common common/
COPY ./teleop app/
WORKDIR /app

EXPOSE 9100

ENV PYTHONPATH "${PYTHONPATH}:/common"
