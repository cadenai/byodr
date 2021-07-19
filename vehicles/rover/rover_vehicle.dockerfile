FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "pymodbus==2.3.0"

RUN pip install geographiclib

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]