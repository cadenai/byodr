FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "cachetools==2.1.0" && \
  pip install "jsoncomment==0.3.3" && \
  pip install "pymodbus==2.3.0" && \
  pip install "requests==2.24.0" && \
  pip install "pytest==4.6.11"

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]