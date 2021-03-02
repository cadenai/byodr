# docker build -f jp43-nano-cp36-cuda.dockerfile -t centipede2donald/nvidia-jetson:jp43-nano-cp36-cuda .
FROM centipede2donald/nvidia-jetson:jp43-nano-cp36-base

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
    rm -rf /usr/local/cuda-10.0/doc &&\
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /