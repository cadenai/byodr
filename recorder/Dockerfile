FROM byodr/ubuntu_bionic

COPY ./common common/
COPY ./recorder app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENV PYTHONPATH "${PYTHONPATH}:/common"