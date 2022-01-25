# Sourced from https://github.com/balena-io-examples
# docker build -f jp42-nano-cp36-base.dockerfile -t centipede2donald/nvidia-jetson:jp42-nano-cp36-base .
FROM balenalib/jetson-nano-ubuntu:bionic as buildstep

WORKDIR /app

# Download with the nvidia sdk manager
COPY /cuda-repo-l4t-10-0*.deb .
COPY /libcudnn7_7*.deb .

ENV DEBIAN_FRONTEND noninteractive

RUN dpkg -i cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb && \
    apt-key add /var/cuda-repo-10-0-local-10.0.326/*.pub && \
    apt-get update && \
    apt-get install -y cuda-toolkit-10-0 && \
    apt-get install -y --no-install-recommends ./libcudnn7_7.6.3.28-1+cuda10.0_arm64.deb && \
    rm -rf *.deb && \
    dpkg --remove cuda-repo-l4t-10-0-local-10.0.326 && \
    dpkg -P cuda-repo-l4t-10-0-local-10.0.326 && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && \
    ldconfig && \
    rm -rf /usr/local/cuda-10.0/doc

FROM balenalib/jetson-nano-ubuntu:bionic as final

COPY --from=buildstep /usr/local/cuda-10.0 /usr/local/cuda-10.0
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

WORKDIR /