SUMMARY = "Stallion MAVLink UART companion and bounded PSO runtime"
LICENSE = "CLOSED & MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
inherit cmake systemd pkgconfig python3-dir
DEPENDS += "json-c vtol-config"

FILESEXTRAPATHS:prepend := "${VTOL_PROVISION_DIR}:"
SRC_URI = "file://vtol-source.tar.gz \
    https://codeload.github.com/mavlink/c_library_v2/tar.gz/e5697c4deaf09068560cb33da2a0283ad6229b3e;downloadfilename=mavlink-e5697c4d.tar.gz;name=mavlink;subdir=mavlink"
SRC_URI[mavlink.sha256sum] = "6b699bd61331d4232931c8e20fcc5b8fdccefabe0fc43b194dfe0598cfd3ba1d"
S = "${WORKDIR}/vtol-source"
OECMAKE_SOURCEPATH = "${S}/companion/allocator"
EXTRA_OECMAKE = "-DBUILD_TESTING=OFF \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DFETCHCONTENT_SOURCE_DIR_MAVLINK=${WORKDIR}/mavlink/c_library_v2-e5697c4deaf09068560cb33da2a0283ad6229b3e"

SYSTEMD_SERVICE:${PN} = "vtol_allocator@vtol.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
RDEPENDS:${PN} += "vtol-config python3-core python3-crypt python3-json python3-jsonschema python3-venv python3-ensurepip python3-pip"
CONFFILES:${PN} += "${sysconfdir}/vtol/pso.json /home/vtol/.local/share/vtol26/config/vehicle.json"
FILES:${PN} += "${systemd_system_unitdir}/vtol_allocator@.service /home/vtol/.local"

do_install:append() {
    install -d ${D}${sysconfdir}/vtol ${D}${systemd_system_unitdir}
    mv ${D}${datadir}/vtol26/companion/mavlink_uart/src/pso/config.json ${D}${sysconfdir}/vtol/pso.json
    install -d ${D}/home/vtol/.local/share/vtol26/config
    install -m 0644 ${S}/config/vehicle.json ${S}/config/validate_vehicle.py ${S}/config/vehicle.schema.json ${S}/config/requirements.txt ${D}/home/vtol/.local/share/vtol26/config/
    chown -R vtol:vtol ${D}/home/vtol
    rm -rf ${D}${datadir}/vtol26
    rmdir --ignore-fail-on-non-empty "${D}${datadir}"
    # CMake installs for the desktop prefix; the image uses Yocto's managed paths.
    if [ "${libdir}/systemd/system" != "${systemd_system_unitdir}" ]; then
        mv ${D}${libdir}/systemd/system/vtol_allocator@.service ${D}${systemd_system_unitdir}/
        rmdir ${D}${libdir}/systemd/system ${D}${libdir}/systemd
    fi
    sed -i \
        -e 's|^ExecStart=.*|ExecStart=${bindir}/vtol_allocator --port /dev/serial0 --baud 230400 --loop-ms 2 --heartbeat-timeout-ms 3000 --pso-config ${sysconfdir}/vtol/pso.json --pso-period-ms 20 --pso-max-particles 96 --pso-max-iterations 15 --pso-external-cycles 2|' \
        ${D}${systemd_system_unitdir}/vtol_allocator@.service
}

# Populate the isolated target environment offline from the image's Python packages.
pkg_postinst_ontarget:${PN}() {
    config_dir=/home/vtol/.local/share/vtol26/config
    ${bindir}/python3 -m venv --without-pip "$config_dir/venv"
    site_packages=$("$config_dir/venv/bin/python3" -I -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
    cp -R ${PYTHON_SITEPACKAGES_DIR}/. "$site_packages/"
    "$config_dir/venv/bin/python3" -I "$config_dir/validate_vehicle.py" "$config_dir/vehicle.json" >/dev/null
    chown -R vtol:vtol "$config_dir/venv"
}
