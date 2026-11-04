# Scarthgap ships networkd in the main systemd package, not systemd-networkd.
SYSTEMD_PACKAGES:append = " ${PN}"
SYSTEMD_SERVICE:${PN}:append = " systemd-networkd.service"
SYSTEMD_AUTO_ENABLE:${PN} = "enable"
