# https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel_20-10.html#rel_20-11
FROM nvcr.io/nvidia/pytorch:20.11-py3

#>>> np.__version__ #'1.19.2'
#>>> cv2.__version__ #'3.4.1'
#>>> torchvision.__version__ #'0.8.0a0'
#>>> torch.__version__ #'1.8.0a0+17f8c32'
#>>> tensorrt.__version__ #'7.2.1.6'
#>>> onnx.__version__ #'1.7.0'
#>>> scipy.__version__ #'1.5.2'
#>>> pytest.__version__ #'6.1.2'
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-zmq \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------
# OpenCV 4.1.1
# ----------------------------------------------------------
RUN pip3 install "opencv-python >=4.1.1,<4.1.2" && \
  pip3 install "scipy==1.5.2" && \
  pip3 install "jsoncomment==0.3.3" && \
  pip3 install "Equation==1.2.1" && \
  pip3 install "pytest==6.1.2"

RUN pip3 install "onnxruntime-gpu==1.7.0"

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.onnx /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"
