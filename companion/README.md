# Raspberry Pi companion application

**Status: not implemented in this repository.** The current source tree has no
Pi application, MAVLink transport configuration, service definition, or
PSO-to-PX4 setpoint adapter. The old `MAVLINK_GUIDE.md` is archived under
`docs/archive/pre-companion-redesign/`; it targets an Arduino STM32F405 and is
not a Raspberry Pi setup guide.

This directory is reserved for the versioned companion application. Before
using it with hardware, the implementation needs to define and document:

1. The Pi/PX4 transport and connection parameters.
2. Heartbeat, telemetry, command, and reconnect handling.
3. How the nine PSO actuator values map into PX4-supported commands.
4. What happens when the Pi, link, or allocator stops responding.
5. Installation, startup, and a restrained bench-test procedure.

The standalone PSO under `../simulation/pso/` is a simulation reference; it is
not yet a flight-control companion application.
