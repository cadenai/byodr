FROM raspbian/stretch

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python-dev \
    python-numpy \
    python-pip \
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
COPY ./raspi app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["sudo", "/pigpio/pigpiod", "-gl"]