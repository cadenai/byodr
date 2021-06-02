# https://github.com/balena-io-playground/jetson-nano-sample-new/blob/master/CUDA/Dockerfile
# docker build -f jp441-nano-cp36-trt.dockerfile -t centipede2donald/nvidia-jetson:jp441-nano-cp36-trt .
FROM balenalib/jetson-nano-ubuntu:bionic

WORKDIR /

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install -y cuda-toolkit-10-2 cuda-samples-10-2 libcudnn8 tensorrt python3-libnvinfer lbzip2 xorg wget tar python3 libegl1 && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/local/cuda-10.2/doc

# Download and install BSP binaries for L4T 32.4.4 (JetPack 4.4.1)
RUN apt-get update && apt-get install -y wget tar lbzip2 python3 libegl1 && \
    wget https://developer.nvidia.com/embedded/L4T/r32_Release_v4.4/r32_Release_v4.4-GMC3/T210/Tegra210_Linux_R32.4.4_aarch64.tbz2 && \
    tar xf Tegra210_Linux_R32.4.4_aarch64.tbz2 && \
    cd Linux_for_Tegra && \
    sed -i 's/config.tbz2\"/config.tbz2\" --exclude=etc\/hosts --exclude=etc\/hostname/g' apply_binaries.sh && \
    sed -i 's/install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/#install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/LC_ALL=C chroot . mount -t proc none \/proc/ /g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/umount ${L4T_ROOTFS_DIR}\/proc/ /g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/chroot . \//  /g' nv_tegra/nv-apply-debs.sh && \
    ./apply_binaries.sh -r / --target-overlay && cd .. \
    rm -rf Tegra210_Linux_R32.4.4_aarch64.tbz2 && \
    rm -rf Linux_for_Tegra && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && ldconfig

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/targets/aarch64-linux/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/extras/CUPTI/lib64

ENV PATH="/usr/local/cuda/bin:${PATH}"

RUN apt-get install -y locales python3 python3-pip git usbutils nano sudo

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ----------------------------------------------------------
# OpenCV 4.1.1
# ----------------------------------------------------------
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc

RUN echo "deb https://repo.download.nvidia.com/jetson/common r32.4 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libopencv-python && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install setuptools Cython wheel
RUN pip3 install numpy==1.19.5
RUN pip3 install "MarkupSafe >=0.9.2, <2"
RUN pip3 install pycuda

RUN apt-get update && apt-get install -y --no-install-recommends \
    make automake gcc g++ cmake \
    libprotobuf-dev protobuf-compiler \
    liblapack-dev \
    libblas-dev \
    gfortran \
    python3-zmq \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install "onnx==1.9.0"

RUN wget https://nvidia.box.com/shared/static/ukszbm1iklzymrt54mgxbzjfzunq7i9t.whl -O onnxruntime_gpu-1.7.0-cp36-cp36m-linux_aarch64.whl && \
    pip3 install onnxruntime_gpu-1.7.0-cp36-cp36m-linux_aarch64.whl && \
    rm -rf onnxruntime_gpu-1.7.0*.whl

RUN ln -s /usr/bin/python3 /usr/bin/python

RUN apt-get update && \
    ldconfig && \
    apt-get install -y --no-install-recommends \
        python3-pip \
	python3-dev \
	libopenblas-dev \
	libopenmpi2 \
        openmpi-bin \
        openmpi-common \
	gfortran && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------
# PyTorch (for JetPack 4.4 DP)
# ----------------------------------------------------------
# Wheels: https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-8-0-now-available/72048
#
ARG PYTORCH_URL=https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl
ARG PYTORCH_WHL=torch-1.7.0-cp36-cp36m-linux_aarch64.whl

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install ${PYTORCH_WHL} --verbose && \
    rm ${PYTORCH_WHL}

# ----------------------------------------------------------
# torchvision
# ----------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    	git \
	build-essential \
	libjpeg-dev \
	zlib1g-dev && \
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/*

ARG TORCHVISION_VERSION=v0.7.0
RUN git clone -b ${TORCHVISION_VERSION} https://github.com/pytorch/vision torchvision && \
    cd torchvision && \
    python3 setup.py install && \
    cd ../ && \
    rm -rf torchvision

RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2-linux-aarch64.tar.gz && \
    tar -zxvf cmake-3.20.2-linux-aarch64.tar.gz && \
    rm cmake-3.20.2-linux-aarch64.tar.gz

ENV PATH="/cmake-3.20.2-linux-aarch64/bin:${PATH}"

RUN git clone --recursive --branch 7.1 https://github.com/onnx/onnx-tensorrt.git && \
    cd onnx-tensorrt && \
    mkdir build && cd build && \
    cmake .. -DTENSORRT_ROOT=/usr/src/tensorrt && make -j8 && \
    make install && ldconfig

RUN python3 -m pip install "scipy==1.5.2" && \
  python3 -m pip install "jsoncomment==0.3.3" && \
  python3 -m pip install "Equation==1.2.1" && \
  python3 -m pip install "pytest==6.1.2" && \
  rm -rf /root/.cache

ENV PYTHONPATH "${PYTHONPATH}:/onnx-tensorrt"
