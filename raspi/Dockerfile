FROM centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.10.2

RUN apt-get update && apt-get install -y \
    usbutils \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN pip install "pymodbus==2.3.0" && \
    pip install "pyusb==1.0.2" && \
    pip install "pytest==4.6.11"

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app