FROM centipede2donald/ros-melodic:python27-opencv32-gstreamer10

COPY ./vehicles/rover/stream*.sh app/
WORKDIR /app

RUN groupadd --gid 1990 --system byodr \
  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

RUN /bin/bash -c "chown byodr:byodr stream*.sh"

USER byodr
