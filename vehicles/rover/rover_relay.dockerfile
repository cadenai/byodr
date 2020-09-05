FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "jsoncomment==0.3.3" && \
  pip install "pyusb==1.0.2" && \
  pip install "pytest==4.6.11"


COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "relay.py", "monitor"]