# Building

## OpenWrt components
```
cd meraki-firmware/openwrt
cp config-x86_64-3.18 .config
make oldconfig
make -j1 BOARD=x86 OPENWRT_EXTRA_BOARD_SUFFIX=_3.18
```

## Kernel
```
cd meraki-firmware/linux-3.18
cp ../openwrt/target/linux/x86_64-3.18/config .config
make CROSS_COMPILE=../openwrt/staging_dir_x86_64_3.18/bin/x86_64-linux-musl- ARCH=x86_64 oldconfig
make CROSS_COMPILE=../openwrt/staging_dir_x86_64_3.18/bin/x86_64-linux-musl- ARCH=x86_64 prepare
touch rootlist
make CROSS_COMPILE=../openwrt/staging_dir_x86_64_3.18/bin/x86_64-linux-musl- ARCH=x86_64 vmlinux
```
