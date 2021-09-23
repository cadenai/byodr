#FROM tensorflow/tensorflow:1.15.0-gpu-py3
#Tensorflow 1.15.4
FROM nvcr.io/nvidia/tensorflow:20.11-tf1-py3

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-setuptools \
    python3-zmq \
    libgl1-mesa-glx \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install "opencv-python >=4.4.0.42,<4.5.0" && \
  pip3 install "numpy<1.19.0,>=1.16.0" && \
  pip3 install "scipy>=1.4.1,<1.5" && \
  pip3 install "jsoncomment==0.3.3" && \
  pip3 install "Equation==1.2.1" && \
  pip3 install "pytest==4.6.11"

#RUN pip3 install "pfilter==0.2.2"

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.pb /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"
