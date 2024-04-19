SOPHGO FFMpeg README
====================

This is SOPHGO FFMpeg project optimized for SOPHGO BM1688 AI chips based on ffmpeg-6.0. It provides hardware accelerations for video codecs 
and image processing filters. Also some enhancement for streaming protocal are included. 

## Changes
With officical FFMpeg, following changes are done in this project. 

** h264_bm, hevc_bm, jpeg_bm codecs are implemented using video hardware engine in SOPHGO Chips for h264/hevc/jpeg decoder or encoder

** bmscale is implemented using image processing hardware in SOPHGO Chips for image processing

** GB2818 streaming protocal support is added in FFMpeg

** Several domenstic CPUs are supported

** Many improvements are customized for Chinese market

## Build Command

download toolchain from linaro gnu toolchain
```base
wget https://releases.linaro.org/components/toolchain/gcc-linaro/6.3-2017.05/gcc-linaro-6.3-2017.05.tar.xz
```

```bash
#!/bin/bash

export PKG_CONFIG_PATH="${PWD}/extern_lib/prebuilt/lib/pkgconfig:$PKG_CONFIG_PATH"
mkdir install

extern_lib_path="${PWD}/extern_lib"

cross_compile_option="--enable-cross-compile --cross-prefix=aarch64-linux-gnu- --target-os=linux --arch=aarch64 --cpu=cortex-a53"

extra_options="--disable-decoder=h264_v4l2m2m --disable-vaapi       \
            --disable-hwaccel=h263_vaapi  --disable-hwaccel=h264_vaapi  --disable-hwaccel=hevc_vaapi    \
            --disable-hwaccel=mjpeg_vaapi --disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi   \
            --disable-hwaccel=vc1_vaapi   --disable-hwaccel=vp8_vaapi   --disable-hwaccel=wmv3_vaapi    \
            --enable-encoder=h264_bm      --enable-encoder=h265_bm      --enable-bmcodec                \
            --enable-decoder=h264_bm      --enable-decoder=hevc_bm  \
            --enable-encoder=jpeg_bm      --enable-decoder=jpeg_bm  \
            --disable-decoder=h263_bm       \
            --disable-decoder=vc1_bm        \
            --disable-decoder=wmv1_bm     --disable-decoder=wmv2_bm     --disable-decoder=wmv3_bm       \
            --disable-decoder=mpeg1_bm    --disable-decoder=mpeg2_bm    --disable-decoder=mpeg4_bm  --disable-decoder=mpeg4v3_bm \
            --disable-decoder=flv1_bm       \
            --disable-decoder=cavs_bm --disable-decoder=avs_bm      \
            --disable-decoder=vp3_bm --disable-decoder=vp8_bm       \
            --enable-zlib --enable-openssl  \
            --disable-encoder=bmx264        \
            --disable-sdl2 --disable-ffplay" 

extra_cflags="-I${extern_lib_path}/hardware/jpeg/include    \
            -I${extern_lib_path}/hardware/video/dec/include \
            -I${extern_lib_path}/hardware/video/enc/include \
            -I${extern_lib_path}/hardware/bmcv/include      \
            -I${extern_lib_path}/3rdparty/osdrv             \
            -I${extern_lib_path}/3rdparty/libbmlib/include  \
            -I${extern_lib_path}/3rdparty/libisp/include    \
            -I${extern_lib_path}/prebuilt/include           \
            -I${extern_lib_path}/prebuilt/include/gbclient  \
            -I${extern_lib_path}/prebuilt/include/freetype2 \
            -DBM1684 -DBM1686 -DBM1688"

extra_ldflags="-L${extern_lib_path}/prebuilt/lib -Wl,-rpath=${extern_lib_path}/prebuilt/lib \
            -L${extern_lib_path}/hardware/video/dec/lib     \
            -L${extern_lib_path}/hardware/video/enc/lib     \
            -L${extern_lib_path}/hardware/jpeg/lib          \
            -L${extern_lib_path}/hardware/bmcv/lib          \
            -L${extern_lib_path}/3rdparty/libbmlib/lib      \
            -L${extern_lib_path}/3rdparty/libisp/lib/soc"

extra_libs="-Wl,--start-group \
            -lrt -lssl -lcrypto -ldl -lresolv -lstdc++ -lgb28181_sip -lm \
            -lbmlib -lbmjpeg -lbmvd -lbmvenc -lbmcv -lispv4l2_helper \
            -lae -laf -lawb -lcvi_bin -lcvi_bin_isp -lcvi_ispd2 -lisp \
            -lisp_algo -lispv4l2_adapter -ljson-c -lsns_full -lcmodel \
            -Wl,--end-group"

./configure \
    --prefix=${PWD}/install             \
    ${cross_compile_option}             \
    --pkg-config=pkg-config             \
    --enable-static --enable-shared     \
    --enable-pic --enable-swscale --enable-libfreetype --enable-libmp3lame \
    ${extra_options}                    \
    --extra_cflags="${extra_cflags}"    \
    --extra-ldflags="${extra_ldflags}"  \
    --extra-libs="${extra_libs}"        \
    --extra-version="sophon-1.0.0"      \

core_number=`cat /proc/cpuinfo | grep "processor" | wc -l`
make -j${core_number}

make install

```

## License

This project is licensed under the LGPL License, Version 3. Please refere to the LICNESE file for detailed information. 

## Authors

Sophgo multimedia team

## Contributing

This project is maintained by Sophgo multimedia team. Welcome to submit codes and patches to us by email to yujing.shen@sophgo.com
