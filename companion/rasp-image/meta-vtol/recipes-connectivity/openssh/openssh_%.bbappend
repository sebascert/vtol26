# Override the sshd_config already fetched/installed by the upstream recipe.
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
