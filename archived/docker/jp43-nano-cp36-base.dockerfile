# Sourced from https://github.com/balena-io-examples/jetson-nano-x11/blob/master/nano_32_3_1/Dockerfile.template
# docker build -f jp43-nano-cp36-base.dockerfile -t centipede2donald/nvidia-jetson:jp43-nano-cp36-base .
FROM balenalib/jetson-nano-ubuntu:bionic

# Prevent apt-get prompting for input
ENV DEBIAN_FRONTEND noninteractive

# Download and install BSP binaries for L4T 32.3.1
RUN apt-get update && apt-get install -y wget tar lbzip2 python3 libegl1 && \
    wget https://developer.nvidia.com/embedded/dlc/r32-3-1_Release_v1.0/t210ref_release_aarch64/Tegra210_Linux_R32.3.1_aarch64.tbz2 && \
    tar xf Tegra210_Linux_R32.3.1_aarch64.tbz2 && \
    cd Linux_for_Tegra && \
    sed -i 's/config.tbz2/config.tbz2 --exclude=etc\/hosts --exclude=etc\/hostname/g' apply_binaries.sh && \
    sed -i 's/install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/#install --owner=root --group=root \"${QEMU_BIN}\" \"${L4T_ROOTFS_DIR}\/usr\/bin\/\"/g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/LC_ALL=C chroot . mount -t proc none \/proc/ /g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/umount ${L4T_ROOTFS_DIR}\/proc/ /g' nv_tegra/nv-apply-debs.sh && \
    sed -i 's/chroot . \//  /g' nv_tegra/nv-apply-debs.sh && \
    ./apply_binaries.sh -r / --target-overlay && cd .. \
    rm -rf Tegra210_Linux_R32.3.1_aarch64.tbz2 && \
    rm -rf Linux_for_Tegra && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && ldconfig

# From https://github.com/BouweCeunen/computer-vision-jetson-nano/blob/master/dockers/l4t/Dockerfile
ARG POWER_MODE=0000
RUN ln -s /etc/nvpmodel/nvpmodel_t210_jetson-nano.conf /etc/nvpmodel.conf && \
    ln -s /etc/systemd/system/nvpmodel.service /etc/systemd/system/multi-user.target.wants/nvpmodel.service && \
    mkdir /var/lib/nvpmodel && \
    echo "/etc/nvpmodel.conf" > /var/lib/nvpmodel/conf_file_path && \
    echo "pmode:${POWER_MODE} fmode:fanNull" > /var/lib/nvpmodel/status