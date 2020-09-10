FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

COPY ./common common/
COPY ./recorder app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]