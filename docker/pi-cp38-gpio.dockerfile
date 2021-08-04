# Must be run from directory root so the python common directory is included in the build context.
# docker build -f docker/pi-cp38-gpio.dockerfile -t centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.20.0 .

FROM raspbian/stretch

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    usbutils \
    build-essential \
    git \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-cachetools \
    python3-setuptools \
    python3-wheel \
    python3-pigpio \
    python3-gpiozero \
    python3-zmq \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

# http://abyz.me.uk/rpi/pigpio/pigpiod.html
RUN git clone https://github.com/joan2937/pigpio.git \
    && cd pigpio \
    && make \
    && make install

RUN pip3 install "pymodbus==2.3.0" && \
    pip3 install "pyusb==1.0.2" && \
    pip3 install "pytest==4.6.11"

RUN pip3 install "odrive==0.5.2"

COPY ./common common/