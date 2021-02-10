FROM centipede2donald/nvidia-jetson:jp42-nano-cp36-tensorflow-115-opencv-440

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.pb /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"
