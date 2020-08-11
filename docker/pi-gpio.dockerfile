# Must be run from directory root so the python common directory is included in the build context.
# docker build -f docker/pi-gpio.dockerfile -t centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.10.1 .

FROM raspbian/stretch

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python-dev \
    python-numpy \
    python-pip \
    python-cachetools \
    python-setuptools \
    python-wheel \
    python-pigpio \
    python-gpiozero \
    python-zmq \
 && rm -rf /var/lib/apt/lists/*

# http://abyz.me.uk/rpi/pigpio/pigpiod.html
RUN git clone https://github.com/joan2937/pigpio.git \
    && cd pigpio \
    && make \
    && make install

COPY ./common common/