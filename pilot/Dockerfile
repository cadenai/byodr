FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "cachetools==2.1.0" && \
  pip install "jsoncomment==0.3.3" && \
  pip install "pid-controller==0.2.0" && \
  pip install "pytest==4.6.11"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]