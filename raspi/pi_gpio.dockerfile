FROM centipede2donald/raspbian-stretch:pigpio-zmq-byodr-0.23.3

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app