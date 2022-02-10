FROM centipede2donald/ubuntu-bionic:python36-opencv32-gstreamer10

#RUN pip3 install "pid-controller==0.2.0"
RUN pip3 install simple-pid
RUN pip3 install "pyusb==1.0.2"

COPY ./common common/
COPY ./pilot app/
WORKDIR /app

ENV PYTHONPATH "${PYTHONPATH}:/common"
