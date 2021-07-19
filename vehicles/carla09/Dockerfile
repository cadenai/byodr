FROM carlasim/carla:0.9.9 AS builder

FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install -r https://raw.githubusercontent.com/carla-simulator/carla/0.9.9/PythonAPI/carla/requirements.txt

COPY ./common common/
COPY ./vehicles/carla09 app/
COPY --from=builder /home/carla/PythonAPI/ PythonAPI/
WORKDIR /app

RUN python -m easy_install --no-deps /PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg

RUN pip install geographiclib

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]