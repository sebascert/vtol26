# Raspberry Pi companion application

A Pi-side C++ UART/MAVLink prototype has been recovered from the USB and copied
to [`mavlink_uart/`](mavlink_uart/). It receives several telemetry messages and
sends a heartbeat, but its command path is still a stub. See that directory's
README for the exact scope and limitations.

The standalone PSO under `../simulation/pso/` is not yet connected to this
prototype. The archived `MAVLINK_GUIDE.md` under
`../docs/archive/pre-companion-redesign/` targets STM32/Arduino and is not a Pi
setup guide.
