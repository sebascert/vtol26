SUMMARY = "Direct Ethernet, SSH identity and companion user"
LICENSE = "CLOSED"
inherit allarch useradd

FILESEXTRAPATHS:prepend := "${VTOL_PROVISION_DIR}:"
SRC_URI = "file://10-vtol-ethernet.network file://vtol-sudo file://authorized_keys"
S = "${WORKDIR}"

USERADD_PACKAGES = "${PN}"
USERADD_PARAM:${PN} = "--create-home --user-group --shell /bin/sh --groups dialout --password '*' vtol"
RDEPENDS:${PN} = "sudo openssh-sshd systemd"

# An invalid password hash leaves SSH key login available without a usable password.
do_install() {
    install -d ${D}${sysconfdir}/systemd/network ${D}${sysconfdir}/sudoers.d
    install -m 0644 ${WORKDIR}/10-vtol-ethernet.network ${D}${sysconfdir}/systemd/network/
    install -m 0440 ${WORKDIR}/vtol-sudo ${D}${sysconfdir}/sudoers.d/vtol
    install -d -m 0700 ${D}/home/vtol/.ssh
    install -m 0600 ${WORKDIR}/authorized_keys ${D}/home/vtol/.ssh/authorized_keys
    chown -R vtol:vtol ${D}/home/vtol
}

FILES:${PN} += "/home/vtol"
CONFFILES:${PN} += "${sysconfdir}/systemd/network/10-vtol-ethernet.network /home/vtol/.ssh/authorized_keys"
