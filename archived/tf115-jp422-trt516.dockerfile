# docker build -f tf115-jp422-trt516.dockerfile -t centipede2donald/nvidia-jetson:jp422-cp36-tf115-trt5.1.6-1 .
FROM bouwe/jetson-nano-l4t-cuda-cudnn-nvinfer-tensorrt-opencv:latest

WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive
ARG HDF5_DIR="/usr/lib/aarch64-linux-gnu/hdf5/serial/"
ARG MAKEFLAGS=-j6

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          python3-pip \
		  python3-dev \
		  gfortran \
		  build-essential \
		  liblapack-dev \
		  libblas-dev \
		  libhdf5-serial-dev \
		  hdf5-tools \
		  libhdf5-dev \
		  zlib1g-dev \
		  zip \
		  libjpeg8-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install setuptools Cython wheel && \
    pip3 install numpy --verbose && \
    pip3 install h5py==2.10.0 --verbose && \
    pip3 install future==0.17.1 \
    mock==3.0.5 \
    keras_preprocessing==1.0.5 \
    keras_applications==1.0.8 \
    gast==0.2.2 \
    futures \
    protobuf \
    pybind11 --verbose

COPY /tensorflow_gpu-1.15* .

# https://developer.download.nvidia.com/compute/redist/jp/v42/tensorflow-gpu/
RUN pip3 install tensorflow_gpu-1.15.0+nv19.11-cp36-cp36m-linux_aarch64.whl && \
    rm -rf tensorflow*.whl && \
    rm -rf /root/.cache

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          python3-zmq \
          python3-pandas \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "jsoncomment==0.3.3"

WORKDIR /

ENTRYPOINT [ "/bin/sh", "-c" ]