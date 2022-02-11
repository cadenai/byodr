FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install simple-pid
RUN pip install "pyusb==1.0.2"
RUN pip install "tornado==4.5.3"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]