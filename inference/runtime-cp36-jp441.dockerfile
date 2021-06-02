FROM centipede2donald/nvidia-jetson:jp441-nano-cp36-oxrt

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.onnx /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"
