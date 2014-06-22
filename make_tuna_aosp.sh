#!/bin/bash

start_time=$(date +%s)

OUT_PATH=~/android/kernel/out

echo "Cleaning build directory..."
rm -rf $OUT_PATH/KERNEL_OBJ
rm -rf $OUT_PATH/system
mkdir $OUT_PATH/KERNEL_OBJ
mkdir $OUT_PATH/system

EXIT=0

THREADS=4
if [ "$#" -eq 1 ]
	then
		THREADS=$1
fi
COMPILER=~/android/arm-cortex_a9-linux-gnueabihf-linaro_4.9.1-2014.06/bin/arm-cortex_a9-linux-gnueabihf-

KERNEL_PATH=~/android/kernel/omap
DEFCONFIG=tuna_defconfig

echo "Building kernel using $THREADS threads..."
echo "*****************************************";

make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER $DEFCONFIG || EXIT=$?
make -j$THREADS -C $KERNEL_PATH O=$OUT_PATH/KERNEL_OBJ ARCH=arm CROSS_COMPILE=$COMPILER zImage || EXIT=$?

echo Exit flag: $EXIT
if [ $EXIT -ge 1 ]; then
   exit;
fi

finish_time=$(date +%s)
echo "Time taken: $((finish_time - start_time)) seconds."


# make flashable zip

# cd into the bundle and use relative paths
cd "${BASH_SOURCE%/*}"

BOOTIMG_TOOLS=./ramdisk-tools

mkdir -p $BOOTIMG_TOOLS/boot/ramdisk
rm $BOOTIMG_TOOLS/boot/*
rm -rf $BOOTIMG_TOOLS/boot/ramdisk/*
cp -rf ./ramdisk_tuna_aosp_kk/* $BOOTIMG_TOOLS/boot/ramdisk/

# repack ramdisk
$BOOTIMG_TOOLS/repack_ramdisk $BOOTIMG_TOOLS/boot/ramdisk

# copy zImage
cp -f $OUT_PATH/KERNEL_OBJ/arch/arm/boot/zImage $BOOTIMG_TOOLS/boot/

# make boot.img
rm $BOOTIMG_TOOLS/ramdisk-kernel/boot.img
$BOOTIMG_TOOLS/mkbootimg --kernel $BOOTIMG_TOOLS/boot/zImage --ramdisk $BOOTIMG_TOOLS/boot/new-ramdisk.cpio.gz -o $BOOTIMG_TOOLS/ramdisk-kernel/boot.img

rm $BOOTIMG_TOOLS/kernel.zip

cd $BOOTIMG_TOOLS/ramdisk-kernel/
zip -r9 ../kernel.zip *

echo "Flashable zip ready at $BOOTIMG_TOOLS/kernel.zip"

exit $EXIT
