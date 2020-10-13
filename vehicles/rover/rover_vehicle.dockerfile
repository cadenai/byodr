FROM centipede2donald/nvidia-jetson:jp43-cp27-tf113-gst1-omx-1b

# jp43-cp27-tf113-gst1-omx-1b is based on jp43-cp27-tf113-2
#
# omx installation:
# https://github.com/balena-io-examples/tx2-sample-app/blob/omx_build/Dockerfile.gstomx
# without libpng12-dev and libjasper-dev
# aclocal to 1.15 in configure script (https://github.com/google/autofdo/issues/38)
# copy from jetson host /usr/lib/aarch64-linux-gnu/libgstnvegl-1.0.so.0
#
# then everything from ubuntu-gstreamer dockerfile
# and pip install "pymodbus==2.3.0"

COPY ./common common/
COPY ./vehicles/rover app/
WORKDIR /app

ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/lib/gstreamer-1.0"
ENV GST_PLUGIN_PATH "/usr/local/lib/gstreamer-1.0"

ENV PYTHONPATH "${PYTHONPATH}:/common"

CMD ["python", "app.py"]