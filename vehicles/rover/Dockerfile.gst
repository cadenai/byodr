FROM centipede2donald/ros-melodic:python27-opencv32-gstreamer10

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

RUN groupadd --gid 1990 --system byodr \
  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

RUN /bin/bash -c "chown byodr:byodr stream*.*"

USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"
