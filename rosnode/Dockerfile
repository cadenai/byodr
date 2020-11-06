FROM ros:foxy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-zmq \
    nano \
    wget

COPY ./common common/
COPY ./rosnode app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python3", "app.py"]