# docker build -f <name> -t centipede2donald/nvidia-jetson:jp42-python27-opencv32-tensorflow113 .
FROM centipede2donald/ubuntu-bionic:python27-opencv32

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    libhdf5-serial-dev \
    hdf5-tools \
    libhdf5-dev \
    zlib1g-dev zip \
    libjpeg8-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Origin https://developer.download.nvidia.com/compute/redist/jp/v411/tensorflow-gpu
COPY /tensorflow_gpu-1.13.0* /

ENV HDF5_DIR=/usr/lib/aarch64-linux-gnu/hdf5/serial

RUN pip install /tensorflow_gpu-1.13.0rc0+nv19.2-cp27-cp27mu-linux_aarch64.whl

RUN pip install --upgrade numpy

WORKDIR /
