# Flash the Pixhawk 6C

Keep propellers removed and battery and actuator power disconnected.

The firmware applies the TELEM2 connection settings automatically. No manual
serial configuration is needed after flashing.

1. Install the [shared vehicle configuration](../../config/README.md) first.
   Then open a terminal in the project root and build the firmware:

   ```sh
   ./controller/build.sh firmware
   ```

   Wait for the build to finish successfully before continuing.

2. Open **QGroundControl**, then connect the Pixhawk to your host computer by
   USB.
3. Click the **Q icon → Configure → Firmware**.
4. Select the **Pixhawk**, then click **Flash**.
5. Unplug and reconnect the USB cable as instructed on screen.
6. Select **PX4**, enable **Advanced settings**, choose **Custom Firmware
   file…**, then click **OK**. In the file picker, select the file below. The
   path is relative to the project root:

   ```text
   controller/build/firmware/px4_fmu-v6c_default.px4
   ```

7. Wait until QGroundControl reports completion and reconnects to the Pixhawk.
   **Keep USB connected throughout the update.**

## Check the Pixhawk firmware

With the Pixhawk connected by USB, open **Q icon → Analyze → MAVLink Console**
and run:

```text
ver all
```

Check that **HW arch** identifies `FMU_V6C`.

Then, in a terminal on your host computer, from the project root, run:

```sh
git -C controller rev-parse HEAD
```

Compare that output with **PX4 git-hash** in `ver all`. The hashes should match.
If you committed changes after building, rebuild and flash again.

Uncommitted edits do not change the Git hash, so matching hashes alone do not
prove that those edits were included. Always rebuild after source changes.
