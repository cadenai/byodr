# docker build -f docker/Dockerfile.ubuntu_opencv -t centipede2donald/ubuntu-bionic:python27-opencv32 .
# docker buildx build --platform linux/arm64,linux/amd64 --push -f docker/Dockerfile.ubuntu_opencv -t centipede2donald/ubuntu-bionic:python27-opencv32 .
FROM ubuntu:bionic

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python-dev \
    python-opencv \
    python-pip \
    python-setuptools \
    python-wheel \
 && rm -rf /var/lib/apt/lists/*

# Included here to avoid the long build times on arm64 child images when missing wheels.
RUN pip install "pyzmq >=18.1, <19.0" \
  && pip install "pandas >=0.22, <1.0"