# Linux IIO Driver for MMA8452Q Accelerometer

This repository contains a Linux Industrial I/O driver for the **MMA8452Q** accelerometer connected through the **I2C** interface.
The project also includes a Device Tree overlay and a Python user-space application for testing, visualization and CSV export.

The project was developed as part of a bachelor's thesis.

---

## Project Overview

The system is organized as follows:

```text
MMA8452Q accelerometer
        ↓ I2C
Linux kernel driver
        ↓
Industrial I/O subsystem
        ↓
/sys/bus/iio/devices/iio:deviceX
/dev/iio:deviceX
        ↓
Python user-space application
```

The driver communicates with the sensor through the Linux I2C subsystem and exposes the acceleration data through the Linux IIO subsystem.

---

## Hardware Compatibility

Tested hardware:

```text
Board: Raspberry Pi 4 Model B
Sensor: MMA8452Q accelerometer
Interface: I2C
I2C bus: i2c1
I2C address: 0x1c
Interrupt pin: GPIO17
```

Device Tree interrupt configuration:

```dts
interrupt-parent = <&gpio>;
interrupts = <17 8>;
```

Meaning:

```text
17 = GPIO17
8  = active LOW interrupt level
```

---

## Software Compatibility

Tested software environment:

```text
Linux distribution: Armbian / Ubuntu-based system
Architecture: aarch64
Kernel: Linux kernel for Raspberry Pi / bcm2711
Kernel module type: external loadable kernel module
```

Required packages:

```bash
sudo apt update
sudo apt install build-essential device-tree-compiler linux-headers-$(uname -r) python3
```

Optional packages for testing:

```bash
sudo apt install i2c-tools
```

---

## Repository Structure

Recommended structure:

```text
mma8452q-linux-iio-driver/
├── driver/
│   ├── mma8452_iio.c
│   └── Makefile
├── device-tree/
│   └── mma8452q-overlay.dts
├── user-space/
│   └── mma8452q_gui.py
├── docs/
│   └── Licenta.pdf
│   
└── README.md
```

---

## Device Tree Overlay

The sensor is declared through a Device Tree overlay.

Example overlay node:

```dts
mma8452q@1c {
    compatible = "nxp,mma8452q";
    reg = <0x1c>;

    interrupt-parent = <&gpio>;
    interrupts = <17 8>;

    status = "okay";
};
```

### Compile the overlay

```bash
dtc -@ -I dts -O dtb -o mma8452q-overlay.dtbo mma8452q-overlay.dts
```

### Install the overlay

```bash
sudo cp mma8452q-overlay.dtbo /boot/firmware/overlays/
```

### Enable the overlay

Edit the Raspberry Pi boot configuration file:

```bash
sudo nano /boot/firmware/config.txt
```

Add at the end:

```text
dtoverlay=mma8452q-overlay
```

Then reboot:

```bash
sudo reboot
```

---

## I2C Verification

After reboot, check if the sensor is visible on the I2C bus:

```bash
sudo i2cdetect -y 1
```

The expected address is:

```text
0x1c
```

Check the `WHO_AM_I` register:

```bash
sudo i2cget -y 1 0x1c 0x0d
```

Expected result:

```text
0x2a
```

Meaning:

```text
0x1c = I2C address of the sensor
0x0d = internal WHO_AM_I register address
0x2a = expected MMA8452Q identification value
```

---

## Build the Kernel Module

Go to the driver directory:

```bash
cd driver
```

Build the module:

```bash
make
```

The build process generates:

```text
mma8452_iio.ko
```

This is the loadable kernel module.

---

## Load the Driver

Load the driver manually:

```bash
sudo insmod mma8452_iio.ko
```

Check if the module is loaded:

```bash
lsmod | grep mma8452
```

Check kernel messages:

```bash
dmesg | grep mma8452
```

Unload the module if needed:

```bash
sudo rmmod mma8452_iio
```

---

## IIO Device Verification

After the driver is loaded, check the IIO devices:

```bash
ls /sys/bus/iio/devices/
```

Expected entries may include:

```text
iio:device0
trigger0
```

Check the device name:

```bash
cat /sys/bus/iio/devices/iio:device0/name
```

Expected result:

```text
mma8452q
```

The trigger name can be checked with:

```bash
cat /sys/bus/iio/devices/trigger0/name
```

Expected result:

```text
mma8452q-trigger
```

---

## Reading Raw Acceleration Values

The driver exposes direct raw readings through sysfs.

Read X axis:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
```

Read Y axis:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_y_raw
```

Read Z axis:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_z_raw
```

These values are raw 12-bit acceleration values converted by the driver into signed integer values.

---

## Reading the Scale

The acceleration scale is exposed through:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_scale
```

The physical acceleration in `g` is calculated as:

```text
acceleration_g = raw_value × scale
```

Example:

```text
x_g = x_raw × in_accel_scale
```

---

## Available Measurement Ranges

Check the available scale values:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_scale_available
```

Typical values:

```text
0.000976562 0.001953125 0.003906250
```

Meaning:

```text
0.000976562 → ±2g
0.001953125 → ±4g
0.003906250 → ±8g
```

---

## Change Measurement Range

Set ±2g:

```bash
echo 0.000976562 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_scale
```

Set ±4g:

```bash
echo 0.001953125 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_scale
```

Set ±8g:

```bash
echo 0.003906250 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_scale
```

Internally, writing to `in_accel_scale` calls the driver's `write_raw()` function and modifies the MMA8452Q `XYZ_DATA_CFG` register.

---

## Sampling Frequency

Read the current sampling frequency:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
```

Check available frequencies:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency_available
```

Typical values:

```text
1.56 6.25 12.5 50 100 200 400 800
```

Set sampling frequency to 100 Hz:

```bash
echo 100 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
```

Set sampling frequency to 400 Hz:

```bash
echo 400 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
```

Internally, writing to `in_accel_sampling_frequency` calls the driver's `write_raw()` function and modifies the MMA8452Q `CTRL_REG1` register.

---

## IIO Buffer Mode

The driver supports triggered buffer acquisition.

Buffer mode uses:

```text
/sys/bus/iio/devices/iio:device0/buffer/
/sys/bus/iio/devices/iio:device0/scan_elements/
/sys/bus/iio/devices/iio:device0/trigger/current_trigger
/dev/iio:device0
```

The buffer sample format is:

```text
[ X_raw | Y_raw | Z_raw | padding | timestamp ]
  2B      2B      2B      2B        8B
```

One sample has 16 bytes.

---

## Configure Buffer Manually

Set the device path:

```bash
DEV=/sys/bus/iio/devices/iio:device0
```

Disable buffer before configuration:

```bash
echo 0 | sudo tee $DEV/buffer/enable
```

Enable scan elements:

```bash
echo 1 | sudo tee $DEV/scan_elements/in_accel_x_en
echo 1 | sudo tee $DEV/scan_elements/in_accel_y_en
echo 1 | sudo tee $DEV/scan_elements/in_accel_z_en
echo 1 | sudo tee $DEV/scan_elements/in_timestamp_en
```

Set buffer length:

```bash
echo 16 | sudo tee $DEV/buffer/length
```

Attach trigger:

```bash
echo mma8452q-trigger | sudo tee $DEV/trigger/current_trigger
```

Enable buffer:

```bash
echo 1 | sudo tee $DEV/buffer/enable
```

Read binary samples:

```bash
sudo dd if=/dev/iio:device0 bs=16 count=10 | hexdump -C
```

Disable buffer after use:

```bash
echo 0 | sudo tee $DEV/buffer/enable
echo "" | sudo tee $DEV/trigger/current_trigger
```

---

## Buffer Data Flow

The buffer acquisition flow is:

```text
MMA8452Q data-ready event
        ↓
INT pin / GPIO17 interrupt
        ↓
Linux IRQ
        ↓
mma8452_irq_thread()
        ↓
iio_trigger_poll()
        ↓
IIO trigger
        ↓
mma8452_trigger_handler()
        ↓
I2C read of X/Y/Z values
        ↓
iio_push_to_buffers_with_timestamp()
        ↓
/dev/iio:device0
```

The timestamp is handled automatically by IIO through:

```c
iio_pollfunc_store_time
```

---

## User-space Python Application

The Python application does not access the I2C hardware directly.

It uses the files exposed by the IIO driver:

```text
/sys/bus/iio/devices/iio:deviceX/
/dev/iio:deviceX
```

The application performs:

```text
automatic IIO device detection
raw X/Y/Z reading
raw-to-g conversion
measurement range configuration
sampling frequency configuration
software offset calibration
live graph display
buffer reading
CSV export
```

Run the application:

```bash
python3 user-space/mma8452q_gui.py
```

---

## How the Python Application Reads Values

The Python application reads sysfs files, equivalent to terminal `cat` commands.

Example terminal command:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
```

Python equivalent:

```python
with open("/sys/bus/iio/devices/iio:device0/in_accel_x_raw", "r") as f:
    value = f.read()
```

For writing configuration values, terminal command:

```bash
echo 100 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
```

Python equivalent:

```python
with open("/sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency", "w") as f:
    f.write("100")
```

For buffer reading, the application opens `/dev/iio:deviceX` in binary mode:

```python
with open("/dev/iio:device0", "rb") as f:
    data = f.read(16)
```

---

## Validation Commands

Check IIO device:

```bash
cat /sys/bus/iio/devices/iio:device0/name
```

Read values:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
cat /sys/bus/iio/devices/iio:device0/in_accel_y_raw
cat /sys/bus/iio/devices/iio:device0/in_accel_z_raw
```

Check scale:

```bash
cat /sys/bus/iio/devices/iio:device0/in_accel_scale
```

Change range and verify:

```bash
echo 0.003906250 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_scale
cat /sys/bus/iio/devices/iio:device0/in_accel_scale
```

Change frequency and verify:

```bash
echo 100 | sudo tee /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
cat /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency
```

Read buffer:

```bash
sudo dd if=/dev/iio:device0 bs=16 count=10 | hexdump -C
```

---

## Notes

* `/sys/bus/iio/devices/iio:deviceX` is used for configuration and direct reading.
* `/dev/iio:deviceX` is used for binary buffer reading.
* `in_accel_scale` is used for raw-to-g conversion.
* `in_accel_sampling_frequency` controls the sensor output data rate.
* The IIO buffer must be disabled before changing scan elements or buffer configuration.
* The application should read from the buffer fast enough to avoid losing samples.

---

## License

This project is provided for educational and research purposes.
