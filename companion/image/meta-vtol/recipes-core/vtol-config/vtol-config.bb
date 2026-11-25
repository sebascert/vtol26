SUMMARY = "Direct Ethernet and companion login"
LICENSE = "CLOSED"
inherit allarch useradd

SRC_URI = "file://10-vtol-ethernet.network file://vtol-sudo"
S = "${WORKDIR}"

USERADD_PACKAGES = "${PN}"
# Default bench login: vtol / vtol; useradd requires a crypt password hash.
VTOL_PASSWORD_HASH = "\$6\$vtol26\$tgGqOdbsv.ld5DXJsAhg7KNZJzEwe3UixOyqsC5ZkDwfrylti8GFwAgCwMtfIk0WDXCZy1kKI5l9PktOylEDo1"
USERADD_PARAM:${PN} = "--create-home --user-group --shell /bin/sh --groups dialout --password '${VTOL_PASSWORD_HASH}' vtol"
RDEPENDS:${PN} = "sudo openssh-sshd systemd"

do_install() {
    install -d ${D}${sysconfdir}/systemd/network
    install -d -m 0750 ${D}${sysconfdir}/sudoers.d
    install -m 0644 ${WORKDIR}/10-vtol-ethernet.network ${D}${sysconfdir}/systemd/network/
    install -m 0440 ${WORKDIR}/vtol-sudo ${D}${sysconfdir}/sudoers.d/vtol
    install -d -m 0755 ${D}/home/vtol
    chown -R vtol:vtol ${D}/home/vtol
}

FILES:${PN} += "/home/vtol"
CONFFILES:${PN} += "${sysconfdir}/systemd/network/10-vtol-ethernet.network"
