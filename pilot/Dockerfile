FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "pid-controller==0.2.0"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]