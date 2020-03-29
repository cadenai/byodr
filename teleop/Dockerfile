FROM centipede2donald/ubuntu-bionic:python27-opencv32

COPY ./common common/
COPY ./teleop app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

# https://github.com/docker/compose/issues/3270. Named volumes are initialized from the first container that uses them.
RUN mkdir /byodr && chown -R app:app /byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

EXPOSE 9100

CMD ["python", "app.py", "--port=9100"]