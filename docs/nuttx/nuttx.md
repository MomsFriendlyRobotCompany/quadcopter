![](https://raw.githubusercontent.com/apache/nuttx/master/Documentation/_static/NuttX320.png)

# Nuttx

```
brew install flock
brew install gcc-arm-embedded
pip install kconfiglib
# brew install x86_64-elf-gcc  # Used by simulator
# brew install u-boot-tools  # Some platform integrate with u-boot
```

```
git clone -b 1.1.2 https://github.com/raspberrypi/pico-sdk.git
export PICO_SDK_PATH=<absolute_path_to_pico-sdk_directory>

git clone https://github.com/apache/nuttx.git nuttx
git clone https://github.com/apache/nuttx-apps.git apps

cd nuttx

make distclean
./tools/configure.sh raspberrypi-pico-w:nsh
make menuconfig
make
```

```
% ./tools/configure.sh -L | grep pico
  raspberrypi-pico:lcd1602
  raspberrypi-pico:waveshare-lcd-1.3
  raspberrypi-pico:usbmsc
  raspberrypi-pico:nshsram
  raspberrypi-pico:nsh-flash
  raspberrypi-pico:nsh
  raspberrypi-pico:audiopack
  raspberrypi-pico:composite
  raspberrypi-pico:st7735
  raspberrypi-pico:displaypack
  raspberrypi-pico:spisd
  raspberrypi-pico:smp
  raspberrypi-pico:enc28j60
  raspberrypi-pico:ssd1306
  raspberrypi-pico:usbnsh
  raspberrypi-pico:waveshare-lcd-1.14
  raspberrypi-pico-w:lcd1602
  raspberrypi-pico-w:waveshare-lcd-1.3
  raspberrypi-pico-w:usbmsc
  raspberrypi-pico-w:nshsram
  raspberrypi-pico-w:nsh-flash
  raspberrypi-pico-w:nsh
  raspberrypi-pico-w:audiopack
  raspberrypi-pico-w:composite
  raspberrypi-pico-w:st7735
  raspberrypi-pico-w:displaypack
  raspberrypi-pico-w:spisd
  raspberrypi-pico-w:smp
  raspberrypi-pico-w:enc28j60
  raspberrypi-pico-w:ssd1306
  raspberrypi-pico-w:usbnsh
  raspberrypi-pico-w:waveshare-lcd-1.14
  raspberrypi-pico-w:telnet
```

```
% make
Create version.h
LN: platform/board to /Users/kevin/tmp/nuttx/apps/platform/dummy
Register: hello
Register: nsh
Register: sh
Register: getprime
Register: ostest
/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/bin/../lib/gcc/arm-none-eabi/13.2.1/../../../../arm-none-eabi/bin/ld: warning: rp2040_boot_stage2.elf has a LOAD segment with RWX permissions
CPP:  /Users/kevin/tmp/nuttx/nuttx/boards/arm/rp2040/raspberrypi-pico-w/scripts/raspberrypi-pico-flash.ld-> /Users/kevin/tmp/nuttx/nuttx/boards/arm/rp2040/raspbLD: nuttx                                            
Generating: nuttx.uf2
tools/rp2040/elf2uf2 nuttx nuttx.uf2;
Done.
```
