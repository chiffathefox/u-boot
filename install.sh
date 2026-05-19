#!/bin/bash

set -xe

make CC=aarch64-linux-gnu-gcc-10 CROSS_COMPILE=aarch64-linux-gnu- -j "$(nproc)"
mkbootimg --kernel u-boot.bin -o boot.img
adb wait-for-recovery
adb push boot.img /tmp
adb shell "dd if=/tmp/boot.img of=/dev/block/platform/13d60000.ufs/by-name/boot"
adb shell "sync"
adb shell "twrp reboot"
