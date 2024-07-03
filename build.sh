#!/bin/bash

export PKG_CONFIG_PATH=../prebuilt/lib/pkgconfig:$PKG_CONFIG_PATH

jpu_path="../bmvid/jpeg/driver/release"
vpu_path="../bmvid/video/provider/cnm/release/soc_$1_$2"
echo jpu_path=${jpu_path}
echo vpu_path=${vpu_path}

./configure \
    --cross-prefix=aarch64-linux-gnu- --enable-cross-compile \
    --target-os=linux --arch=aarch64 --cpu=cortex-a53 \
    --pkg-config=pkg-config \
    --disable-static --enable-shared --enable-pic \
    --enable-swscale --enable-libfreetype --enable-libx264 --enable-gpl \
    --disable-decoder=h264_v4l2m2m \
    --extra-ldflags="-L${vpu_path} -lbmvideo -lrt" \
    --extra-libs="${jpu_path}/libbmjpu.a" \
    --extra-cflags="-I${vpu_path}/ -I${jpu_path}/" \
    --extra-ldflags="-L../prebuilt/lib -Wl,-rpath=../prebuilt/lib"

core_number=`cat /proc/cpuinfo | grep "processor" | wc -l`
make -j${core_number}
