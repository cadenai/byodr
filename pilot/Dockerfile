FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

RUN pip3 install simple-pid
RUN pip3 install "pyusb==1.0.2"
RUN pip3 install "tornado==6.1"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python3", "app.py"]