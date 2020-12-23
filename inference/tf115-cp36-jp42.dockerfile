FROM centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorrt516-tensorflow115-opencv440

#RUN pip3 install "pfilter==0.2.2"

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.pb /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"
