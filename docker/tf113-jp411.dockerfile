# docker build -f tf113-jp411.dockerfile -t centipede2donald/nvidia-jetson:jp411-cp27-tf113-1 .
FROM w251/tensorrt:dev-xavier-4.1.1_b57

ENV DEBIAN_FRONTEND noninteractive

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    libhdf5-serial-dev \
    hdf5-tools \
    libhdf5-dev \
    build-essential \
    python-dev \
    python-opencv \
    python-pandas \
    python-pip \
    python-setuptools \
    python-wheel \
    python-zmq \
    python-h5py \
    zlib1g-dev zip \
    libjpeg8-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    wget && \
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/*

# From https://developer.download.nvidia.com/compute/redist/jp/v411/tensorflow-gpu
COPY /tensorflow_gpu-1.13.0* .

RUN pip install "jsoncomment==0.3.3" && \
  pip install "numpy==1.16.6" && \
  pip install "pytest==4.6.11"

RUN pip install tensorflow_gpu-1.13.0rc0+nv19.2-cp27-cp27mu-linux_aarch64.whl && \
    rm -rf tensorflow*.whl

ENV LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-10.0/targets/aarch64-linux/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra

WORKDIR /