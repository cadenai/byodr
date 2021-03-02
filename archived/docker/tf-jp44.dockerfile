# docker build -f <name> -t centipede2donald/nvidia-jetson:jp44-r32.4.3-tf1.15-py3 .
FROM nvcr.io/nvidia/l4t-tensorflow:r32.4.3-tf1.15-py3

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-pandas \
    python3-pip \
    python3-zmq \
    wget \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /
