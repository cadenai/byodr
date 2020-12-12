# docker build -f tf115-cp36-jp44.dockerfile -t centipede2donald/l4t:32.4.3-nano-jetpack-4.4-tf-1.15.4-nv20.11-cp36-tensorrt-opencv-4.4 .
FROM centipede2donald/l4t:32.4.3-nano-jetpack-4.4-tf-1.15.4-nv20.11-cp36-tensorrt

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    make automake gcc g++ \
    build-essential \
    gfortran \
    python3-pip \
    python3-setuptools \
    python3-zmq \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    python3-dev \
    libgl1-mesa-glx \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*


RUN pip3 install "opencv-python >=4.4.0.42,<4.5.0" && \
  pip3 install "numpy<1.19.0,>=1.16.0" && \
  pip3 install "scipy>=1.4.1,<1.5" && \
  pip3 install "jsoncomment==0.3.3" && \
  pip3 install "Equation==1.2.1" && \
  pip3 install "pytest==4.6.11"

WORKDIR /