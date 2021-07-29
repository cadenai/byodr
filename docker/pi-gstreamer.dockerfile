# Must be run from directory root so the python common directory is included in the build context.
# docker build -f docker/pi-gstreamer.dockerfile -t centipede2donald/raspbian-stretch:gst-omx-rpi-0.50.0 .

FROM mmastrac/gst-omx-rpi

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    nano \
    v4l-utils \
    python3-gi \
    python3-numpy \
    python3-six \
    python3-tornado \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

WORKDIR /

COPY ./common common/

CMD ["sleep", "infinity"]