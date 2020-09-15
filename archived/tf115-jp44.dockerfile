# docker build -f tf115-jp44.dockerfile -t centipede2donald/nvidia-jetson:jp44-cp36-tf115-2 .
FROM nvcr.io/nvidia/l4t-base:r32.4.3

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

# https://developer.download.nvidia.com/compute/redist/jp/v44/tensorflow/
COPY /tensorflow-1.15* .

RUN pip3 install tensorflow-1.15.3+nv20.8-cp36-cp36m-linux_aarch64.whl && \
    rm -rf tensorflow*.whl && \

ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

WORKDIR /
