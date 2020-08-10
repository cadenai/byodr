FROM carlasim/carla:0.9.9 AS builder

FROM centipede2donald/ubuntu-bionic:python27-opencv32

RUN pip install -r https://raw.githubusercontent.com/carla-simulator/carla/0.9.9/PythonAPI/carla/requirements.txt

COPY ./common common/
COPY ./vehicles/carla09 app/
COPY --from=builder /home/carla/PythonAPI/ PythonAPI/
WORKDIR /app

RUN python -m easy_install --no-deps /PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg

#RUN groupadd --gid 1990 --system byodr \
#  && useradd --uid 1990 --gid 1990 --system --create-home --shell /bin/bash byodr

#USER byodr

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]