FROM centipede2donald/ubuntu-bionic:python27-opencv32

RUN pip install "cachetools >=2.1, <3.0" && \
  pip install "jsoncomment >=0.3, <1.0" && \
  pip install "pid-controller >=0.2, <1.0" && \
  pip install "pytest"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

#RUN groupadd --gid 1990 --system byodr \
#  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

#USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]