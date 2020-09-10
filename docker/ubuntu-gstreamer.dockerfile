# For docker hub to have this as a cross platform image build and push it at the same time.
# docker buildx build --platform linux/arm64,linux/amd64 --push -f ubuntu-gstreamer.dockerfile -t centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10 .
FROM ubuntu:bionic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python-dev \
    python-opencv \
    python-pandas \
    python-pip \
    python-setuptools \
    python-wheel \
    python-zmq \
    nano \
    openssh-client \
    lm-sensors \
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
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN pip install "cachetools==2.1.0" && \
  pip install "jsoncomment==0.3.3" && \
  pip install "requests==2.24.0" && \
  pip install "pytest==4.6.11"
