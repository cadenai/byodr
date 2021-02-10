# docker build -f tf115-cp36-jp44.dockerfile -t centipede2donald/nvidia-jetson:nano-jp44-tf-1.15.4-nv20.11-cp36-opencv-4.4 .
FROM balenalib/jetson-nano-ubuntu:bionic as buildstep

WORKDIR /app

# Download with the nvidia sdk manager
COPY /cuda-repo-l4t-10-2*.deb .
COPY /libcudnn8_*.deb .

ENV DEBIAN_FRONTEND noninteractive

RUN dpkg -i cuda-repo-l4t-10-2-local-10.2.89_1.0-1_arm64.deb && \
    apt-key add /var/cuda-repo-10-2-local-10.2.89/*.pub && \
    apt-get update && \
    apt-get install -y cuda-toolkit-10-2 ./libcudnn8_8.0.0.180-1+cuda10.2_arm64.deb && \
    rm -rf *.deb && \
    dpkg --remove cuda-repo-l4t-10-2-local-10.2.89 && \
    dpkg -P cuda-repo-l4t-10-2-local-10.2.89 && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && \
    ldconfig

RUN rm -rf /usr/local/cuda-10.2/doc

FROM balenalib/jetson-nano-ubuntu:bionic as final

COPY --from=buildstep /usr/local/cuda-10.2 /usr/local/cuda-10.2
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
    rm -rf *.tbz2

RUN apt-get update && apt-get install -y --no-install-recommends \
    make automake gcc g++ \
    build-essential \
    python3-dev \
    python3-pip \
    python3-setuptools \
    libhdf5-dev \
    libhdf5-serial-dev \
    python3-h5py \
    gfortran \
    hdf5-tools \
    libblas-dev \
    liblapack-dev \
    pkg-config \
    python3-zmq \
    libgl1-mesa-glx \
    && \
    python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade setuptools && \
    python3 -m pip install --no-cache-dir --upgrade wheel && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install "absl-py>=0.7.0" && \
    python3 -m pip install "astor>=0.6.0" && \
    python3 -m pip install "gast==0.2.2" && \
    python3 -m pip install "google-pasta>=0.1.6" && \
    python3 -m pip install "keras-applications>=1.0.8" && \
    python3 -m pip install "keras-preprocessing>=1.0.5" && \
    python3 -m pip install "protobuf>=3.6.1" && \
    python3 -m pip install "numpy<1.19.0,>=1.16.0" && \
    python3 -m pip install "opt-einsum>=2.3.2" && \
    python3 -m pip install "six>=1.10.0" && \
    python3 -m pip install "tensorboard<1.16.0,>=1.15.0" && \
    python3 -m pip install "tensorflow-estimator==1.15.1" && \
    python3 -m pip install "termcolor>=1.1.0" && \
    python3 -m pip install "wrapt>=1.11.1" && \
    python3 -m pip install "grpcio>=1.8.6"

RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

RUN python3 -m pip wheel --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 tensorflow==1.15.4+nv20.11

RUN pip3 install "opencv-python >=4.4.0.42,<4.5.0" && \
  pip3 install "numpy<1.19.0,>=1.16.0" && \
  pip3 install "scipy>=1.4.1,<1.5" && \
  pip3 install "jsoncomment==0.3.3" && \
  pip3 install "Equation==1.2.1" && \
  pip3 install "pytest==4.6.11"

RUN ls -1 -d *.whl | xargs python3 -m pip install --no-cache-dir

RUN rm -rf /root/.cache && rm -rf *.whl

ENV LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-10.2/targets/aarch64-linux/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra

WORKDIR /