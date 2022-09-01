SOPHGO FFMpeg README
====================

This is SOPHGO FFMpeg project optimized for SOPHGO BM1682/1684/1684x AI chips. It provides hardware accelerations for video codecs 
and image processing filters. Also some enhancement for streaming protocal are included. 

## Changes
With officical FFMpeg, following changes are done in this project. 

** h264_bm, hevc_bm, jpeg_bm codecs are implemented using video hardware engine in SOPHGO Chips for h264/hevc/jpeg decoder or encoder

** bmscale is implemented using image processing hardware in SOPHGO Chips for image processing

** GB2818 streaming protocal support is added in FFMpeg

** Several domenstic CPUs are supported

** Many improvements are customized for Chinese market

## Build Command

### x86_64 linux with local gcc
    1. export PKG_CONFIG_PATH=./extern_lib/prebuilt/x86_64/lib/pkgconfig:$PKG_CONFIG_PATH$
    2. configure
    ./configure --target-os=linux \
        --pkg-config=pkg-config \
        --optflags=" "  \
        --enable-libmp3lame \
        --enable-static --enable-shared --enable-pic \
        --enable-swscale --enable-libfreetype \
        --enable-bmcodec --disable-encoder=bmx264  \
        --enable-encoder=jpeg_bm --enable-decoder=jpeg_bm \
        --disable-decoder=vc1_bm --disable-decoder=wmv1_bm --disable-decoder=wmv2_bm \
        --disable-decoder=wmv3_bm --disable-decoder=mpeg1_bm --disable-decoder=mpeg2_bm --disable-decoder=mpeg4_bm  --disable-decoder=mpeg4v3_bm --disable-decoder=flv1_bm \
        --disable-decoder=h263_bm --disable-decoder=cavs_bm --disable-decoder=avs_bm --disable-decoder=vp3_bm --disable-decoder=vp8_bm --enable-openssl \
        --disable-decoder=h264_v4l2m2m  \
        --disable-vaapi --disable-hwaccel=h263_vaapi  --disable-hwaccel=h264_vaapi --disable-hwaccel=hevc_vaapi --disable-hwaccel=mjpeg_vaapi \
        --disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi --disable-hwaccel=vc1_vaapi --disable-hwaccel=vp8_vaapi \
        --disable-hwaccel=vp8_vaapi --disable-hwaccel=wmv3_vaapi \
        --extra-ldflags="-L./extern_lib/bm_hardware_accele/decode_x86_64/lib -L./extern_lib/3rdparty/libbmcv/lib/pcie -Wl,-rpath=./extern_lib/3rdparty/libbmcv/lib/pcie -L./extern_lib/prebuilt/x86_64/lib -Wl,-rpath=./extern    _lib/prebuilt/x86_64/lib" \
        --extra-libs="-lm -lbmion -lbmvpulite -lbmvpuapi -lbmvppapi -lbmlib -lbmcv -lbmjpulite -lbmjpuapi -lbmvideo -lrt -lssl -lcrypto -ldl -lresolv -lstdc++ -lgb28181_sip" \
        --extra-cflags="-DBM1684 -I./extern_lib/3rdparty/libbmcv/include -DBM_PCIE_MODE=1 -I./extern_lib/bmvid/cnm/driver/release/pcie_bm1684_asic/ -I./extern_lib/bm_hardware_accele/decode_x86_64/include/ -I./extern_lib/pr    ebuilt/include/freetype2 -I./extern_lib/prebuilt/include/ -I./extern_lib/prebuilt/include/gbclient"$
	4. make and install to DESTDIR

### x86_64 linux with x86_64-linux-gcc 5.4.0
	1. download toolchain 
		wget https://toolchains.bootlin.com/downloads/releases/toolchains/x86-64-core-i7/tarballs/x86-64-core-i7--glibc--stable-2017.05-toolchains-1-1.tar.bz2
    2. export PKG_CONFIG_PATH=./extern_lib/prebuilt/x86_64/lib/pkgconfig:$PKG_CONFIG_PATH$
    3. configure
    ./configure --cross-prefix=x86_64-linux- --enable-cross-compile --target-os=linux --arch=x86_64 \
        --pkg-config=pkg-config \
        --optflags=" "  \
        --enable-libmp3lame \
        --enable-static --enable-shared --enable-pic \
        --enable-swscale --enable-libfreetype \
        --enable-bmcodec --disable-encoder=bmx264  \
        --enable-encoder=jpeg_bm --enable-decoder=jpeg_bm \
        --disable-decoder=vc1_bm --disable-decoder=wmv1_bm --disable-decoder=wmv2_bm \
        --disable-decoder=wmv3_bm --disable-decoder=mpeg1_bm --disable-decoder=mpeg2_bm --disable-decoder=mpeg4_bm  --disable-decoder=mpeg4v3_bm --disable-decoder=flv1_bm \
        --disable-decoder=h263_bm --disable-decoder=cavs_bm --disable-decoder=avs_bm --disable-decoder=vp3_bm --disable-decoder=vp8_bm --enable-openssl \
        --disable-decoder=h264_v4l2m2m  \
        --disable-vaapi --disable-hwaccel=h263_vaapi  --disable-hwaccel=h264_vaapi --disable-hwaccel=hevc_vaapi --disable-hwaccel=mjpeg_vaapi \
        --disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi --disable-hwaccel=vc1_vaapi --disable-hwaccel=vp8_vaapi \
        --disable-hwaccel=vp8_vaapi --disable-hwaccel=wmv3_vaapi \
        --extra-ldflags="-L./extern_lib/bm_hardware_accele/decode_x86_64/lib -L./extern_lib/3rdparty/libbmcv/lib/pcie -Wl,-rpath=./extern_lib/3rdparty/libbmcv/lib/pcie -L./extern_lib/prebuilt/x86_64/lib -Wl,-rpath=./extern_lib/prebuilt/x86_64/lib" \
        --extra-libs="-lm -lbmion -lbmvpulite -lbmvpuapi -lbmvppapi -lbmlib -lbmcv -lbmjpulite -lbmjpuapi -lbmvideo -lrt -lssl -lcrypto -ldl -lresolv -lstdc++ -lgb28181_sip" \
        --extra-cflags="-DBM1684 -I./extern_lib/3rdparty/libbmcv/include -DBM_PCIE_MODE=1 -I./extern_lib/bmvid/cnm/driver/release/pcie_bm1684_asic/ -I./extern_lib/bm_hardware_accele/decode_x86_64/include/ -I./extern_lib/prebuilt/include/freetype2 -I./extern_lib/prebuilt/include/ -I./extern_lib/prebuilt/include/gbclient"
	4. make and install to DESTDIR
	
### SOPHGO soc with aarch64-linux-gnu-gcc
	1. download toolchain from linaro gnu toolchain
	   wget https://releases.linaro.org/components/toolchain/gcc-linaro/6.3-2017.05/gcc-linaro-6.3-2017.05.tar.xz
    2. export PKG_CONFIG_PATH=./extern_lib/prebuilt/lib/pkgconfig:$PKG_CONFIG_PATH$
    3. configure
	./configure --enable-cross-compile --cross-prefix=aarch64-linux-gnu- --target-os=linux --arch=aarch64 \
        --pkg-config=pkg-config \
        --optflags=" "  \
        --enable-static --enable-shared --enable-pic \
        --enable-swscale --enable-libfreetype \
        --enable-libmp3lame \
        --enable-bmcodec --disable-encoder=bmx264  \
        --disable-sdl2 --disable-ffplay \
        --enable-encoder=jpeg_bm --enable-decoder=jpeg_bm \
        --disable-decoder=vc1_bm --disable-decoder=wmv1_bm --disable-decoder=wmv2_bm \
        --disable-decoder=wmv3_bm --disable-decoder=mpeg1_bm --disable-decoder=mpeg2_bm --disable-decoder=mpeg4_bm  --disable-decoder=mpeg4v3_bm --disable-decoder=flv1_bm \
        --disable-decoder=h263_bm --disable-decoder=cavs_bm --disable-decoder=avs_bm --disable-decoder=vp3_bm --disable-decoder=vp8_bm --enable-openssl \
        --disable-decoder=h264_v4l2m2m  \
        --disable-vaapi --disable-hwaccel=h263_vaapi  --disable-hwaccel=h264_vaapi --disable-hwaccel=hevc_vaapi --disable-hwaccel=mjpeg_vaapi \
        --disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi --disable-hwaccel=vc1_vaapi --disable-hwaccel=vp8_vaapi \
        --disable-hwaccel=vp8_vaapi --disable-hwaccel=wmv3_vaapi \
        --extra-ldflags="-L./extern_lib/bm_hardware_accele/decode_arm64/lib -L./extern_lib/prebuilt/lib -Wl,-rpath=./extern_lib/prebuilt/lib"\
        --extra-libs="-lm -lbmion -lbmvpulite -lbmvpuapi -lbmvppapi -lbmjpulite -lbmcv -lbmlib -lbmjpuapi -lbmvideo -lrt -lssl -lcrypto -ldl -lresolv -lstdc++ -lgb28181_sip" \
        --extra-cflags="-DBM1684 -I./extern_lib/3rdparty/libbmcv/include -DBM_PCIE_MODE=1 -I./extern_lib/bmvid/cnm/driver/release/pcie_bm1684_asic/ -I./extern_lib/bm_hardware_accele/decode_arm64/include/ -I./extern_lib/pre    built/include/freetype2 -I./extern_lib/prebuilt/include/ -I./extern_lib/prebuilt/include/gbclient"$
	4. make and install to DESTDIR
		
## License

This project is licensed under the LGPL License, Version 3. Please refere to the LICNESE file for detailed information. 

## Authorts

	- xun.li  
	- tao.han
	- yujing.shen
	- huaishan.yuan
    - yuyuan.yang
    - xin.guo
    - yu.yang
    - mingxi.shen
    - haotian.luo
	
## Contributing

This project is maintained by Sophgo multimedia team. Welcome to submit codes and patches to us by email to yujing.shen@sophgo.com
