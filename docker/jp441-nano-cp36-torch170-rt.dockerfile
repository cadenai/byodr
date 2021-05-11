# Sourced from https://raw.githubusercontent.com/balena-io-examples/balena-ros2-CUDA-trt-pose-estimation/main/ros/Dockerfile.
# docker build -f jp441-nano-cp36-torch170-rt.dockerfile -t centipede2donald/nvidia-jetson:jp441-nano-cp36-torch170-rt .
FROM balenalib/jetson-nano-ubuntu:bionic

# Prevent apt-get prompting for input
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y wget tar lbzip2 \
    && \
    wget https://developer.nvidia.com/embedded/L4T/r32_Release_v4.4/r32_Release_v4.4-GMC3/T210/Tegra210_Linux_R32.4.4_aarch64.tbz2 && \
    tar xf Tegra210_Linux_R32.4.4_aarch64.tbz2 && \
    cd Linux_for_Tegra && \
    sed -i 's/config.tbz2\"/config.tbz2\" --exclude=etc\/hosts --exclude=etc\/hostname/g' apply_binaries.sh && \
    sed -i 's/install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/#install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/chroot . \//  /g' nv_tegra/nv-apply-debs.sh && \
    ./apply_binaries.sh -r / --target-overlay && cd .. \
    rm -rf Tegra210_Linux_R32.4.2_aarch64.tbz2 && \
    rm -rf Linux_for_Tegra && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && ldconfig

ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra
ENV UDEV=1

RUN apt-get install -y locales python3 python3-pip git usbutils nano sudo

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ----------------------------------------------------------
# install OpenCV (with GStreamer support)
# ----------------------------------------------------------
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc

RUN echo "deb https://repo.download.nvidia.com/jetson/common r32.4 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libopencv-python && apt-get clean


# ----------------------------------------------------------
# PyCUDA
# ----------------------------------------------------------
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
RUN echo "$PATH" && echo "$LD_LIBRARY_PATH"

RUN apt-get update && apt-get install -y python3 python3-pip
RUN apt-get install -y libopenblas-base libopenmpi-dev cuda-toolkit-10-2 && apt-get clean
RUN apt-get update && apt-get install -y libcudnn8 python3-libnvinfer-dev cuda-compiler-10-2 && apt-get clean
#RUN apt-get update && apt-get install -y -o Dpkg::Options::=--force-confdef nvidia-l4t-gstreamer libgstreamer1.0-0 gstreamer1.0-plugins-base gobject-introspection gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-tools && apt-get clean
RUN pip3 install pycuda --verbose


# ----------------------------------------------------------
# Gst Python
# ----------------------------------------------------------
RUN apt update && apt install python3-gi python3-dev python3-gst-1.0 -y



# ----------------------------------------------------------
# PyTorch Installations
# ----------------------------------------------------------
#
# install prerequisites (many of these are for numpy)
#
ENV PATH="/usr/local/cuda-10.2/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-10.2/lib64:/usr/local/cuda-10.2/extras/CUPTI/lib64:${LD_LIBRARY_PATH}"

RUN apt-get update && \
    ldconfig && \
    apt-get install -y --no-install-recommends \
        python3-pip \
	python3-dev \
	libopenblas-dev \
	libopenmpi2 \
        openmpi-bin \
        openmpi-common \
	gfortran

RUN pip3 install setuptools Cython wheel
RUN pip3 install numpy --verbose


# ----------------------------------------------------------
# PyTorch (for JetPack 4.4 DP)
# ----------------------------------------------------------
#  PyTorch v1.2.0 https://nvidia.box.com/shared/static/lufbgr3xu2uha40cs9ryq1zn4kxsnogl.whl (torch-1.2.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.3.0 https://nvidia.box.com/shared/static/017sci9z4a0xhtwrb4ps52frdfti9iw0.whl (torch-1.3.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.4.0 https://nvidia.box.com/shared/static/c3d7vm4gcs9m728j6o5vjay2jdedqb55.whl (torch-1.4.0-cp36-cp36m-linux_aarch64.whl)
#  PyTorch v1.5.0 https://nvidia.box.com/shared/static/3ibazbiwtkl181n95n9em3wtrca7tdzp.whl (torch-1.5.0-cp36-cp36m-linux_aarch64.whl)
#

ARG PYTORCH_URL=https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl
ARG PYTORCH_WHL=torch-1.7.0-cp36-cp36m-linux_aarch64.whl

RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${PYTORCH_URL} -O ${PYTORCH_WHL} && \
    pip3 install ${PYTORCH_WHL} --verbose && \
    rm ${PYTORCH_WHL}


# ----------------------------------------------------------
# torchvision 0.4
# ----------------------------------------------------------
ARG TORCHVISION_VERSION=v0.7.0
#ARG PILLOW_VERSION="pillow<7"
ARG TORCH_CUDA_ARCH_LIST="7.2"

RUN printenv && echo "torchvision version = $TORCHVISION_VERSION" && echo "pillow version = $PILLOW_VERSION" && echo "TORCH_CUDA_ARCH_LIST = $TORCH_CUDA_ARCH_LIST"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    	git \
	build-essential \
	libjpeg-dev \
	zlib1g-dev && \
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone -b ${TORCHVISION_VERSION} https://github.com/pytorch/vision torchvision && \
    cd torchvision && \
    python3 setup.py install && \
    cd ../ && \
    rm -rf torchvision


# ----------------------------------------------------------
# torch2trt installations
# ----------------------------------------------------------
RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt && \
    cd torch2trt && \
    python3 setup.py install --plugins

