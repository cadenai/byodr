# docker build -f jp43-nano-cp36-tf115.dockerfile -t centipede2donald/nvidia-jetson:jp43-nano-cp36-tensorflow-115 .
FROM centipede2donald/nvidia-jetson:jp43-nano-cp36-cuda

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    make automake gcc g++ \
    pkg-config \
    libhdf5-serial-dev \
    hdf5-tools \
    libhdf5-dev \
    build-essential \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-h5py \
    zlib1g-dev zip \
    libjpeg8-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    wget && \
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf *.tbz2

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade setuptools && \
    python3 -m pip install --no-cache-dir --upgrade wheel

ENV LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-10.0/targets/aarch64-linux/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra

RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

RUN pip3 install -U numpy grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor protobuf \
    keras-applications keras-preprocessing wrapt google-pasta

# From https://developer.download.nvidia.com/compute/redist/jp
COPY /tensorflow_gpu-1.15* .

RUN pip3 install tensorflow_gpu-1.15.0+nv20.1-cp36-cp36m-linux_aarch64.whl && \
    rm -rf tensorflow*.whl && \
    rm -rf /root/.cache
