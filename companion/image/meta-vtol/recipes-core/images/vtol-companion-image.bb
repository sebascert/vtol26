SUMMARY = "Raspberry Pi 4B MAVLink companion with direct Ethernet SSH"
LICENSE = "CLOSED"
inherit core-image

IMAGE_FEATURES += "ssh-server-openssh"
IMAGE_INSTALL:append = " vtol-allocator vtol-config sudo iproute2 iputils-ping systemd"
IMAGE_FSTYPES += "wic.bz2 wic.bmap"
IMAGE_ROOTFS_EXTRA_SPACE = "262144"
