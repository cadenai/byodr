FROM centipede2donald/ubuntu-bionic:python27-opencv32

RUN pip install "jsoncomment >=0.3, <1.0" && \
  pip install "pytest"

COPY ./common common/
COPY ./recorder app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]