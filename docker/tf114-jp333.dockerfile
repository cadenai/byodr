# docker build -f tf114-jp333.dockerfile -t centipede2donald/nvidia-jetson:jp333-cp27-tf114-1 .
FROM balenalib/jetson-nano-ubuntu:bionic as buildstep

WORKDIR /app

# Download with the nvidia sdk manager
COPY /cuda-repo-*.deb .
COPY /libcudnn7_*.deb .

ENV DEBIAN_FRONTEND noninteractive

RUN dpkg -i cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb && \
    apt-key add /var/cuda-repo-9-0-local/*.pub && \
    apt-get update && \
    apt-get install -y cuda-toolkit-9-0 ./libcudnn7_7.1.5.14-1+cuda9.0_arm64.deb && \
    rm -rf *.deb && \
    dpkg --remove cuda-repo-l4t-9-0-local && \
    dpkg -P cuda-repo-l4t-9-0-local && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && \
    ldconfig

RUN rm -rf /usr/local/cuda-9.0/doc

FROM balenalib/jetson-nano-ubuntu:bionic as final

COPY --from=buildstep /usr/local/cuda-9.0 /usr/local/cuda-9.0
COPY --from=buildstep /usr/lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu
COPY --from=buildstep /usr/local/lib /usr/local/lib

WORKDIR /app

COPY /nvidia_drivers.tbz2 .
COPY /config.tbz2 .

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install lbzip2 -y && \
    tar xjf nvidia_drivers.tbz2 -C / && \
    tar xjf config.tbz2 -C / --exclude=etc/hosts --exclude=etc/hostname && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && ldconfig && \
    apt-get install -y --no-install-recommends \
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
    rm -rf /var/lib/apt/lists/* && \
    rm -rf *.tbz2

RUN pip install "jsoncomment==0.3.3" && \
  pip install "pytest==4.6.11"

# From https://developer.download.nvidia.com/compute/redist/jp/
COPY /tensorflow_gpu-1.14.0* .

RUN pip install tensorflow_gpu-1.14.0+nv19.9-cp27-cp27mu-linux_aarch64.whl && \
    rm -rf tensorflow*.whl && \
    rm -rf /root/.cache

WORKDIR /