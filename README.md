# NMEA2000 Battery Voltage, Engine RPM, Fuel Level and Exhaust Temp Sender
This repository is my version of [NMEA2000-Data-Sender](https://github.com/AK-Homberger/NMEA2000-Data-Sender)


The project using the NMEA2000 libraries from Timo Lappalainen: https://github.com/ttlappalainen.
And the  [NMEA2000_esp32_c3](https://github.com/mosvov/NMEA2000_esp32_c3) with TWAI CAN driver to support ESP32 C3

![PXL_20230206_015807297](https://user-images.githubusercontent.com/2290361/217978254-8c23e53c-211e-4df4-afc5-85893417b98c.jpg)



How to build PlatformIO based project
=====================================

1. [Install PlatformIO Core](https://docs.platformio.org/page/core.html) Or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode) 
2. Download [development platform with examples](https://github.com/platformio/platform-espressif32/archive/develop.zip)
3. Extract ZIP archive
4. Run these commands:

```shell

# Change directory to example
$ cd platform-espressif32/examples/espidf-aws-iot

# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Clean build files
$ pio run --target clean
```
