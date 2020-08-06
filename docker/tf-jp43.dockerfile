# docker build -f <name> -t centipede2donald/nvidia-jetson:jp43-python27-opencv32-tensorflow113 .
FROM nvcr.io/nvidia/l4t-base:r32.3.1

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Download with the nvidia sdk manager
COPY /cuda-repo-l4t-10-0* /
COPY /libcudnn7* /

RUN dpkg -i /cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb
RUN apt-key add /var/cuda-repo-10-0-local-10.0.326/7fa2af80.pub

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-10-0 \
    cuda-cublas-10-0 \
    /libcudnn7_7.5.0.56-1+cuda10.0_arm64.deb \
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
    wget \
 && rm -rf /var/lib/apt/lists/*

# Origin https://developer.download.nvidia.com/compute/redist/jp/v411/tensorflow-gpu
COPY /tensorflow_gpu-1.13.0* /

RUN pip install /tensorflow_gpu-1.13.0rc0+nv19.2-cp27-cp27mu-linux_aarch64.whl
RUN pip install --upgrade numpy

WORKDIR /
