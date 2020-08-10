# docker build -f <name> -t centipede2donald/ros-melodic:python27-opencv32-gstreamer10 .
FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python-dev \
    python-opencv \
    python-pip \
    python-setuptools \
    python-wheel \
    python-zmq \
    wget

RUN apt-get update && apt-get install -f -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-0 \
    python-gi \
 && rm -rf /var/lib/apt/lists/*
