# docker build -f jp42-nano-cp36-tf115.dockerfile -t centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorrt516-tensorflow115 .
FROM centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorrt516

ENV DEBIAN_FRONTEND noninteractive

RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

RUN pip3 install -U numpy grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor protobuf \
    keras-applications keras-preprocessing wrapt google-pasta

# From https://developer.download.nvidia.com/compute/redist/jp
COPY /tensorflow_gpu-1.15* .

RUN pip3 install tensorflow_gpu-1.15.0+nv19.11-cp36-cp36m-linux_aarch64.whl && \
    rm -rf tensorflow*.whl && \
    rm -rf /root/.cache
