# Installation (Ubuntu 26.04 Host)

## Dependencies
```bash
sudo apt update
sudo apt install gcc-aarch64-linux-gnu adb mkbootimg git \
  bc bison build-essential coccinelle \
  device-tree-compiler dfu-util efitools flex gdisk graphviz imagemagick \
  libgnutls28-dev libguestfs-tools libncurses-dev \
  libpython3-dev libsdl2-dev libssl-dev lz4 lzma lzma-alone openssl \
  pkg-config python3 python3-asteval python3-coverage python3-filelock \
  python3-pkg-resources python3-pycryptodome python3-pyelftools \
  python3-pytest python3-pytest-xdist python3-sphinxcontrib.apidoc \
  python3-sphinx-rtd-theme python3-subunit python3-testtools \
  python3-venv swig uuid-dev
```

## Compilation and Installation
```bash
git clone https://github.com/chiffathefox/u-boot -b exynos9820
cd u-boot
make CC=aarch64-linux-gnu-gcc CROSS_COMPILE=aarch64-linux-gnu- exynos9820_defconfig
make CC=aarch64-linux-gnu-gcc CROSS_COMPILE=aarch64-linux-gnu- -j "$(nproc)"
mkbootimg --kernel u-boot.bin -o boot.img
adb wait-for-recovery
adb push boot.img /tmp
adb shell dd if=/tmp/boot.img of=/dev/block/platform/13d60000.ufs/by-name/boot
```

