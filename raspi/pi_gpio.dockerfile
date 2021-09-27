FROM centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.22.1

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app