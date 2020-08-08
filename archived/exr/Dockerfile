FROM centipede2donald/ros-melodic:python27-opencv32-gstreamer10

#
# ueye
# on the host disable the usb daemon: sudo update-rc.d ueyeusbdrc disable
# in the container the ueyeusbdrc must be script started since containers do not use initd nor systemd.
# use privilegd, or capabilities
# container mount volume /dev/ueye:/dev/ueye
#
# pcan
# use privilegd, or capabilities
# container mount pcan devices as volumes and provide access to the compiled libpcan* via LD_LIBRARY_PATH
#

RUN apt-get update && apt-get install -y --no-install-recommends \
    usbutils \
    udev \
    libpopt-dev \
    net-tools \
    iproute2 \
    kmod \
 && rm -rf /var/lib/apt/lists/*

COPY ./common common/
COPY ./docker/deps deps/
COPY ./vehicles/exr app/

WORKDIR /app
RUN /bin/cp exr_entry.sh /exr_entry.sh \
  && /bin/bash -c "chmod +x /exr_entry.sh"

RUN pip install --no-cache-dir -r requirements.txt

RUN /deps/ueye_4.92.0.0_arm64.run --auto \
  && ueyesetup -i usb

RUN useradd --uid 1000 --gid 1000 --system --create-home --shell /bin/bash app

ENTRYPOINT ["/exr_entry.sh"]
