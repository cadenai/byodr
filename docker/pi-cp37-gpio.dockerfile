# Must be run from directory root so the python common directory is included in the build context.
# docker build -f docker/pi-cp37-gpio.dockerfile -t centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.25.0 .

#FROM raspbian/stretch
FROM balenalib/rpi-raspbian:stretch

ENV UDEV=1

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

RUN pip3 install "crccheck"

RUN git clone https://github.com/LiamBindle/PyVESC.git \
    && cd PyVESC \
    && sed -i 's/return f"{self.comm_fw_version}.{self.fw_version_major}.{self.fw_version_minor}"/return "{self.comm_fw_version}.{self.fw_version_major}.{self.fw_version_minor}"/' pyvesc/VESC/messages/getters.py \
    && python3 setup.py build \
    && python3 setup.py install

COPY ./common common/