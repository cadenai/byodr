FROM centipede2donald/ros-melodic:python27-opencv32-gstreamer10

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

RUN groupadd --gid 1000 --system app \
  && useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENV PYTHONPATH "${PYTHONPATH}:/common"

ENTRYPOINT ["/ros_entrypoint.sh"]