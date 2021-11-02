# Must be run from directory root so the python common directory is included in the build context.
# docker build -f docker/pi-cp37-gpio.dockerfile -t centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.23.0 .

FROM raspbian/stretch

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    usbutils \
    i2c-tools \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-cachetools \
    python3-setuptools \
    python3-wheel \
    python3-pigpio \
    python3-gpiozero \
    python3-smbus \
    python3-zmq \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

# http://abyz.me.uk/rpi/pigpio/pigpiod.html
RUN git clone https://github.com/joan2937/pigpio.git \
    && cd pigpio \
    && make \
    && make install

RUN pip3 install "pymodbus==2.3.0" && \
    pip3 install "pyusb==1.1.1" && \
    pip3 install "pytest==4.6.11"

RUN pip3 install "pythoncrc" && \
    pip3 install "pyvesc"

COPY ./common common/