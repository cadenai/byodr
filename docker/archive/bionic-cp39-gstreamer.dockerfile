# For docker hub to have this as a cross platform image build and push it at the same time.
# docker buildx build --platform linux/arm64,linux/amd64 --push -f bionic-cp39-gstreamer.dockerfile -t centipede2donald/ubuntu-bionic:python39-opencv32-gstreamer10 .
FROM ubuntu:bionic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    nano \
    openssh-client \
    lm-sensors \
    wget \
    software-properties-common

RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y python3.9 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --all

RUN rm -rf /usr/local/lib/python3.6 && \
    rm -rf /usr/lib/python3.6 && \
    rm -rf /usr/bin/python3.6 && \
    rm -rf /usr/share/doc/python3.6 && \
    rm -rf /usr/share/lintian/overrides/python3.6 && \
    rm -rf /usr/share/binfmts/python3.6 && \
    rm -rf /etc/python3.6

RUN apt-get update && apt-get install -f -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-0 \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*


#RUN apt-get install -y --no-install-recommends \
#    python3-dev \
#    python3-opencv \
#    python3-pandas \
#    python3-pip \
#    python3-setuptools \
#    python3-wheel \
#    python3-zmq
#     python3-gi \
#    python3-gst-1.0 \
#RUN pip3 install "pyarrow==7.0.*" && \
#    pip3 install "cachetools==2.1.0" && \
#    pip3 install "jsoncomment==0.3.3" && \
#    pip3 install "requests==2.24.0" && \
#    pip3 install "pytest==4.6.11"
