FROM centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.10.1

RUN apt-get update && apt-get install -y \
    usbutils \
 && rm -rf /var/lib/apt/lists/*

RUN pip install "pymodbus >=2.3.0, <3.0" && \
    pip install "pyusb >=1.0.2, <2.0"

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app