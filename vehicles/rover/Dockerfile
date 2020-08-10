FROM centipede2donald/ros-melodic:python27-opencv32-gstreamer10

RUN pip install "cachetools >=2.1, <3.0" && \
  pip install "jsoncomment >=0.3, <1.0" && \
  pip install "pymodbus >=2.3.0, <3.0" && \
  pip install "pyusb >=1.0.2, <2.0" && \
  pip install "requests >=2.10, <3.0"

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

#RUN groupadd --gid 1990 --system byodr \
#  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

#USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["python", "app.py"]