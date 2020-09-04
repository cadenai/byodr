FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "jsoncomment >=0.3, <1.0" && \
  pip install "pyusb >=1.0.2, <2.0" && \
  pip install "pytest"


COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "relay.py", "monitor"]