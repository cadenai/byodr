FROM centipede2donald/raspbian-stretch:gst-omx-rpi-0.50.1

ENV PYTHONPATH "${PYTHONPATH}:/common"

COPY ./ app/

WORKDIR /app

CMD ["sleep", "infinity"]