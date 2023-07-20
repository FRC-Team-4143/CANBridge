# CANBridge

## Overview

This repository is for the CANBridge software that is run on non-roboRIO platforms.

## Build Requirements

1. Git
2. Visual Studio Code or some other equivalent IDE
3. Gradle Build Tool 
4. Java JDK/JRE

All of these, excluding Git, can be installed and configured with the [WPILib Installer](https://github.com/wpilibsuite/allwpilib/releases), which will include specific versions for building FRC robot code. Gradle needs a C++ compiler, which is included with VS Code. 

## Building and Publishing

Building is done using the Gradle wrapper, `gradlew`. The Gradle wrapper is located in the root of the project, so Gradle commands must be run from there. 

1. Clone this repository and open in VS Code
   - When VS Code first opens, select `Add workspace folder...` underneath `Start` on the Welcome Screen
2. Open the VS Code terminal
   -  `View -> Terminal` or ``Ctrl+` ``
3. Run `./gradlew build` from root

The output is placed at `~\releases\maven\release\com\revrobotics\usb\CANBridge-cpp\<version>\`.

### Publishing a new version

Before publishing a new version, run `./gradlew build` locally to run the tests. GitHub Actions
cannot run the tests because they depend on having a USB CAN device connected.

1. Bump the version number in `publish.gradle` and `CANBridge.json`
2. Commit the version bump
3. Create a new tag named `vX.X.X` at that commit
4. Push the tag to GitHub
5. Wait for the draft release to be created
6. Add release notes to the draft release
7. Publish the draft release

## Linux

This branch is a work in progress. The latest firmware version will work with Linux if the socketCAN and gs_usb drivers are enabled. The following udev rule will work to enable this:

/etc/udev/rules.d/

```
ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="a30e", RUN+="/sbin/modprobe gs_usb" RUN+="/bin/sh -c 'echo 0483 a30e > /sys/bus/usb/drivers/gs_usb/new_id'"
```
Currently does NOT work with SparkMax. 
But DOES work with candleLight_fw usb-to-can devices.

enable the SocketCAN device with:
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000

## Changelog

The SDK Changelog can be viewed with [Changelog.md](Changelog.md).

