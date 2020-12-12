# docker build -f jp42-nano-cp36-tf115-opencv44.dockerfile -t centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorflow-115-opencv-440 .
FROM centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorflow-115

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-zmq \
    libgl1-mesa-glx \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /wheelhouse
ENV PIP_WHEEL_DIR=/wheelhouse
ENV WHEELHOUSE=/wheelhouse
ENV PIP_FIND_LINKS=/wheelhouse

RUN python3 -m pip install "opencv-python >=4.4.0.42,<4.5.0" && \
  python3 -m pip install "numpy<1.19.0,>=1.16.0" && \
  python3 -m pip install "scipy>=1.4.1,<1.5" && \
  python3 -m pip install "jsoncomment==0.3.3" && \
  python3 -m pip install "Equation==1.2.1" && \
  python3 -m pip install "pytest==4.6.11" && \
  rm -rf /root/.cache

#RUN ls -1 -d /wheelhouse/*.whl | xargs python3 -m pip install --no-cache-dir && rm -rf /wheelhouse
RUN rm -rf /wheelhouse
