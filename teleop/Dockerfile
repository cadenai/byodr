FROM centipede2donald/ubuntu-bionic:python27-opencv32

COPY ./common common/
COPY ./teleop app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1990 --system byodr \
  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

EXPOSE 9100

RUN mkdir /byodr && chown -R byodr:byodr /byodr

VOLUME /byodr

USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py", "--port=9100"]