# Flash the Raspberry Pi 4B

The image includes the companion application and its boot configuration.

Power off the Pi before removing its card.

1. Open a terminal in the project folder and build the image:

   ```sh
   ./companion/image/build.sh
   ```

   Wait for the build to finish successfully before continuing.

2. Decompress the disk image:

   ```sh
   IMAGE=companion/image/build/yocto/tmp/deploy/images/raspberrypi4-64/vtol-companion-image-raspberrypi4-64.rootfs.wic.bz2
   bzip2 -t "$IMAGE" &&
   bzip2 -dc "$IMAGE" > companion/image/build/vtol-companion.img
   ```

3. Insert the card into the host computer and identify its device:

   ```sh
   lsblk -e 7 -o NAME,SIZE,MODEL,TRAN,MOUNTPOINTS
   ```

   > [!WARNING]
   > The commands below erase **`/dev/sda`**. Confirm its size and model
   > identify your microSD card, and replace the device names if needed.
   > Selecting the wrong disk can destroy your host computer's operating system
   > and data.

4. Unmount its mounted partitions, then write to the whole card:

   ```sh
   sudo umount /dev/sda1 /dev/sda2
   ```

   Already-unmounted partitions are harmless. Resolve any other error before
   continuing.

   ```sh
   sudo dd if=companion/image/build/vtol-companion.img of=/dev/sda \
     bs=4M status=progress conv=fsync
   ```

   Wait for `dd` to finish successfully.

## Check the written image

Compare the card with the image, then safely disconnect it:

```sh
sudo cmp -n "$(stat -c %s companion/image/build/vtol-companion.img)" \
  companion/image/build/vtol-companion.img /dev/sda &&
echo "Verification passed" &&
udisksctl power-off -b /dev/sda
```

After successful verification and power-off, remove the card reader. Insert the
card into the powered-off Pi, then connect its USB-C power supply.
