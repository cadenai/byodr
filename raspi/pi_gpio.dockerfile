FROM centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.20.0

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app