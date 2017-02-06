# Linux Kernel for ARTIK
## Contents
1. [Introduction](#1-introduction)
2. [Build guide](#2-build-guide)
3. [Update guide](#3-update-guide)

## 1. Introduction
This 'linux-artik' repository is linux kernel source for artik5(artik520),
artik10(artik1020), artik710 and artik530. The base kernel version of artik5
and artik10 is linux-3.10.93 and based on Samsung Exynos kernel.
The artik710 and artik530 kernel is based on is based on linux-4.4.19.

---
## 2. Build guide
### 2.1 Install cross compiler
+ For artik710> You'll need an arm64 cross compiler
```
sudo apt-get install gcc-aarch64-linux-gnu
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget https://releases.linaro.org/components/toolchain/binaries/5.3-2016.05/aarch64-linux-gnu/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu.tar.xz
tar xf gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu.tar.xz
export PATH=~/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu:$PATH
```
You can the path permernently through adding it into ~/.bashrc

+ ARTIK5, ARTIK10 and ARTIK530
```
sudo apt-get install gcc-arm-linux-gnueabihf
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget http://releases.linaro.org/components/toolchain/binaries/latest-5/arm-linux-gnueabihf/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
tar xf gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz
export PATH=~/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin:$PATH
```
You can the path permernently through adding it into ~/.bashrc

### 2.2 Install android-fs-tools
To generate modules.img which contains kernel modules, you can use the make_ext4fs.
```
sudo apt-get install android-tools-fsutils
```

### 2.2 Build the kernel
+ For artik710>
```
make ARCH=arm64 artik710_raptor_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm64 menuconfig
```

```
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image -j4
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- dtbs
mkdir usr/modules
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules_install INSTALL_MOD_PATH=usr/modules INSTALL_MOD_STRIP=1
make_ext4fs -b 4096 -L modules \
	-l 32M usr/modules.img \
	usr/modules/lib/modules/
rm -rf usr/modules
```

+ For artik530>
```
make ARCH=arm artik530_raptor_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm menuconfig
```

```
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
mkdir usr/modules
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules_install INSTALL_MOD_PATH=usr/modules INSTALL_MOD_STRIP=1
make_ext4fs -b 4096 -L modules \
	-l 32M usr/modules.img \
	usr/modules/lib/modules/
rm -rf usr/modules
```

+ For artik5>
```
make ARCH=arm artik5_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm menuconfig
```

```
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- exynos3250-artik5.dtb
./scripts/mk_modules.sh
```

+ For artik10>
```
make ARCH=arm artik10_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm menuconfig
```

```
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- exynos5422-artik10.dtb
./scripts/mk_modules.sh
```

## 3. Update Guide
Copy compiled binaries into your board.

+ For artik710>
```
scp arch/arm64/boot/Image root@{YOUR_BOARD_IP}:/root
scp arch/arm64/boot/dts/nexell/*.dtb root@{YOUR_BOARD_IP}:/root
scp usr/modules.img root@{YOUR_BOARD_IP}:/root
```

+ On your board
```
mount -o remount,rw /boot
cp /root/Image /boot
cp /root/*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p2
sync
reboot
```

+ For artik530>
```
scp arch/arm/boot/zImage root@{YOUR_BOARD_IP}:/root
scp arch/arm/boot/dts/s5p4418*.dtb root@{YOUR_BOARD_IP}:/root
scp usr/modules.img root@{YOUR_BOARD_IP}:/root
```

+ On your board
```
mount -o remount,rw /boot
cp /root/zImage /boot
cp /root/s5p4418*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p2
sync
reboot
```

+ For artik5 and artik10>
```
scp arch/arm/boot/zImage root@{YOUR_BOARD_IP}:/root
scp arch/arm/boot/dts/*.dtb root@{YOUR_BOARD_IP}:/root
scp usr/modules.img root@{YOUR_BOARD_IP}:/root
```

+ On your board
```
mount -o remount,rw /boot
cp /root/zImage /boot
cp /root/*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p2
sync
reboot
```
