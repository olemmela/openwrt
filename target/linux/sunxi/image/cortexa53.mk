#
# Copyright (C) 2013-2016 OpenWrt.org
# Copyright (C) 2016 Yousong Zhou
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Device/sun50i
  SUNXI_DTS_DIR := allwinner/
  KERNEL_LOADADDR:=0x40080000
  KERNEL_NAME := Image
  KERNEL := kernel-bin | lzma | fit lzma $$(DTS_DIR)/$$(SUNXI_DTS).dtb
endef

define Device/sun50i-h5
  SOC := sun50i-h5
  $(Device/sun50i)
endef

define Device/sun50i-a64
  SOC := sun50i-a64
  $(Device/sun50i)
endef

define Device/friendlyarm_nanopi-neo-plus2
  DEVICE_VENDOR := FriendlyARM
  DEVICE_MODEL := NanoPi NEO Plus2
  SUPPORTED_DEVICES:=nanopi-neo-plus2
  $(Device/sun50i-h5)
endef
TARGET_DEVICES += friendlyarm_nanopi-neo-plus2

define Device/friendlyarm_nanopi-neo2
  DEVICE_VENDOR := FriendlyARM
  DEVICE_MODEL := NanoPi NEO2
  SUPPORTED_DEVICES:=nanopi-neo2
  $(Device/sun50i-h5)
endef
TARGET_DEVICES += friendlyarm_nanopi-neo2

define Device/olimex-a64_olinuxino
  DEVICE_VENDOR := Olimex
  DEVICE_MODEL := A64-Olinuxino
  DEVICE_PACKAGES := kmod-rtl8723bs rtl8723bs-firmware
  $(Device/sun50i-a64)
endef
TARGET_DEVICES += olimex-a64_olinuxino

define Device/olimex-a64_olinuxino-emmc
  DEVICE_VENDOR := Olimex
  DEVICE_MODEL := A64-Olinuxino
  DEVICE_VARIANT := eMMC
  DEVICE_PACKAGES := kmod-rtl8723bs rtl8723bs-firmware
  $(Device/sun50i-a64)
endef
TARGET_DEVICES += olimex-a64_olinuxino-emmc

define Device/pine64_pine64-lts
  DEVICE_VENDOR := Pine64
  DEVICE_MODEL := Pine64 LTS
  FILESYSTEMS := squashfs
  IMAGE_SIZE := 15360k
  IMAGES += sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | append-rootfs | pad-rootfs | append-metadata | check-size $$$$(IMAGE_SIZE)
  ARTIFACTS := u-boot-with-spl.bin
  ARTIFACT/u-boot-with-spl.bin := sunxi-uboot | check-size 896k
  $(Device/sun50i-a64)
endef

TARGET_DEVICES += pine64_pine64-lts

define Device/pine64_pine64-plus
  DEVICE_VENDOR := Pine64
  DEVICE_MODEL := Pine64+
  $(Device/sun50i-a64)
endef
TARGET_DEVICES += pine64_pine64-plus

define Device/pine64_sopine-baseboard
  DEVICE_VENDOR := Pine64
  DEVICE_MODEL := SoPine
  $(Device/sun50i-a64)
endef
TARGET_DEVICES += pine64_sopine-baseboard

define Device/xunlong_orangepi-pc2
  DEVICE_VENDOR := Xunlong
  DEVICE_MODEL := Orange Pi PC 2
  $(Device/sun50i-h5)
endef
TARGET_DEVICES += xunlong_orangepi-pc2

define Device/xunlong_orangepi-zero-plus
  DEVICE_VENDOR := Xunlong
  DEVICE_MODEL := Orange Pi Zero Plus
  $(Device/sun50i-h5)
endef
TARGET_DEVICES += xunlong_orangepi-zero-plus
