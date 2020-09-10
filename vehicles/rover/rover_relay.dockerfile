FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "pyusb==1.0.2"

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "relay.py", "monitor"]