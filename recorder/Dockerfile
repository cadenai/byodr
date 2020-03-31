FROM centipede2donald/ubuntu-bionic:python27-opencv32

COPY ./common common/
COPY ./recorder app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1990 --system byodr \
  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]