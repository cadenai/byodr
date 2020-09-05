FROM centipede2donald/ubuntu-bionic:python27-opencv32-gstreamer10

RUN pip install "tornado==4.5.3" && \
  pip install "pytest==4.6.11"

COPY ./common common/
COPY ./teleop app/
WORKDIR /app

EXPOSE 9100

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py", "--port=9100"]