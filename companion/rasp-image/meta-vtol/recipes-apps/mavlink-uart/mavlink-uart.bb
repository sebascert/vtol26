SUMMARY = "Stallion MAVLink UART companion and bounded PSO runtime"
LICENSE = "CLOSED & MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
inherit cmake systemd

FILESEXTRAPATHS:prepend := "${VTOL_PROVISION_DIR}:"
SRC_URI = "file://vtol-source.tar.gz \
    https://codeload.github.com/mavlink/c_library_v2/tar.gz/e5697c4deaf09068560cb33da2a0283ad6229b3e;downloadfilename=mavlink-e5697c4d.tar.gz;name=mavlink;subdir=mavlink"
SRC_URI[mavlink.sha256sum] = "6b699bd61331d4232931c8e20fcc5b8fdccefabe0fc43b194dfe0598cfd3ba1d"
S = "${WORKDIR}/vtol-source"
OECMAKE_SOURCEPATH = "${S}/companion/mavlink_uart"
EXTRA_OECMAKE = "-DBUILD_TESTING=OFF \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DFETCHCONTENT_SOURCE_DIR_MAVLINK=${WORKDIR}/mavlink/c_library_v2-e5697c4deaf09068560cb33da2a0283ad6229b3e"

SYSTEMD_SERVICE:${PN} = "mavlink-uart@vtol.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
RDEPENDS:${PN} += "vtol-config"
CONFFILES:${PN} += "${sysconfdir}/vtol/pso.json"
FILES:${PN} += "${systemd_system_unitdir}/mavlink-uart@.service"

do_install:append() {
    install -d ${D}${sysconfdir}/vtol ${D}${systemd_system_unitdir}
    mv ${D}${datadir}/vtol26/companion/mavlink_uart/src/pso/config.json ${D}${sysconfdir}/vtol/pso.json
    rm -rf ${D}${datadir}/vtol26
    # CMake installs for the desktop prefix; the image uses Yocto's managed paths.
    if [ "${libdir}/systemd/system" != "${systemd_system_unitdir}" ]; then
        mv ${D}${libdir}/systemd/system/mavlink-uart@.service ${D}${systemd_system_unitdir}/
        rmdir ${D}${libdir}/systemd/system ${D}${libdir}/systemd
    fi
    sed -i \
        -e 's|/usr/local/bin/mavlink_uart|${bindir}/mavlink_uart|' \
        -e 's|/usr/local/share/vtol26/companion/mavlink_uart/src/pso/config.json|${sysconfdir}/vtol/pso.json|' \
        ${D}${systemd_system_unitdir}/mavlink-uart@.service
}
