FROM centipede2donald/nvidia-jetson:jp43-cp27-tf113-2

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-scipy \
 && apt-get -y clean && rm -rf /var/lib/apt/lists/*

RUN pip install "Equation==1.2.1"

COPY ./common common/
COPY ./inference app/
WORKDIR /app

COPY ./build/*.pb /models/
COPY ./build/*.ini /models/

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]