FROM ros:melodic AS builder

FROM raspbian/stretch

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python-dev \
    python-pip \
    python-setuptools \
    python-wheel \
    python-gpiozero \
    python-yaml \
    python-roslib \
    python-rosmsg \
    python-rospy \
 && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/ros/melodic/ /opt/ros/melodic/
COPY . app/
WORKDIR /app

RUN pip install --no-cache-dir -r requirements.txt

#RUN useradd --system --create-home --shell /bin/bash pi
#USER pi

ENV PATH "${PATH}:/opt/ros/melodic/bin"
ENV PYTHONPATH "${PYTHONPATH}:/opt/ros/melodic/lib/python2.7/dist-packages"
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/opt/ros/melodic/lib"
ENV ROS_ETC_DIR "/opt/ros/melodic/etc/ros"
ENV ROS_ROOT "/opt/ros/melodic/share/ros"
ENV ROS_PACKAGE_PATH "/opt/ros/melodic/share"
ENV PKG_CONFIG_PATH "/opt/ros/melodic/lib/pkgconfig"
ENV ROS_DISTRO "melodic"

CMD ["python", "app.py"]