#!/bin/bash
export PKG_CONFIG_PATH=../prebuilt/windows/lib/pkgconfig::$PKG_CONFIG_PATH
extra_lib_path="../prebuilt/windows"
extra_bmvid_lib_path="../bmvid/release/lib"
extra_bmlib_path="../bm_opencv/3rdparty/libbmcv"
extra_inc_path="../bmvid"
build_type=$(echo $1 | tr '[A-Z]' '[a-z]')
if [[ "$build_type" =~ "debug" ]]; then
  echo debug
  ./configure \
	--prefix=..\\ffmpeg_install \
	--target_os=mingw64 \
	--disable-x86asm \
	--enable-debug \
	--disable-optimizations \
	--disable-stripping \
	--enable-static --enable-shared --enable-pic \
	--enable-swscale --enable-libfreetype \
	--enable-bmcodec --enable-encoder=bmx264 --enable-encoder=jpeg_bm --enable-decoder=jpeg_bm \
	--disable-sdl2 --disable-ffplay \
	--disable-decoder=vc1_bm --disable-decoder=wmv1_bm --disable-decoder=wmv2_bm --disable-decoder=wmv3_bm --disable-decoder=mpeg1_bm --disable-decoder=mpeg2_bm --disable-decoder=mpeg4_bm --disable-decoder=mpeg4v3_bm --disable-decoder=flv1_bm --disable-decoder=h263_bm --disable-decoder=cavs_bm --disable-decoder=avs_bm --disable-decoder=vp3_bm --disable-decoder=vp8_bm \
	--disable-decoder=h264_v4l2m2m \
	--disable-vaapi --disable-hwaccel=h263_vaapi --disable-hwaccel=h264_vaapi --disable-hwaccel=hevc_vaapi --disable-hwaccel=mjpeg_vaapi \
	--disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi --disable-hwaccel=vc1_vaapi --disable-hwaccel=vp8_vaapi \
	--disable-hwaccel=vp8_vaapi --disable-hwaccel=wmv3_vaapi \
	--extra-libs="-L${extra_lib_path}/lib -L${extra_bmvid_lib_path} -L${extra_bmlib_path}/lib/pcie_windows -lbmjpulite -lbmjpuapi -lbmion -lbmvpuapi -lbmvppapi -lbmvideo -lbmlib -lbmcv -lvpp_cmodel -lharfbuzz -lfreetype -lpng -lz -lbz2 -lgb28181_sip -lws2_32" \
	--extra-cflags="-I${extra_inc_path}/bmcv/include -I${extra_lib_path}/include -I${extra_inc_path}/video/encoder/bm_enc_api/inc -I${extra_inc_path}/video/provider/cnm/encoder/inc -I${extra_inc_path}/video/decoder/bm_dec_api/inc -I${extra_inc_path}/allocator/ion/inc -I${extra_inc_path}/vpp/bmvppapi/inc -I${extra_inc_path}/jpeg/driver/bmjpuapi/inc -I${extra_bmlib_path}/include -I${extra_lib_path}/include/gbclient -DBM1684 -DBM_PCIE_MODE"
elif [[ "$build_type" =~ "release" ]]; then
  echo release
  ./configure \
	--prefix=..\\ffmpeg_install \
	--target_os=mingw64 \
	--disable-x86asm \
	--enable-static --enable-shared --enable-pic \
	--enable-swscale --enable-libfreetype \
	--enable-bmcodec --enable-encoder=bmx264 --enable-encoder=jpeg_bm --enable-decoder=jpeg_bm \
	--disable-sdl2 --disable-ffplay \
	--disable-decoder=vc1_bm --disable-decoder=wmv1_bm --disable-decoder=wmv2_bm --disable-decoder=wmv3_bm --disable-decoder=mpeg1_bm --disable-decoder=mpeg2_bm --disable-decoder=mpeg4_bm --disable-decoder=mpeg4v3_bm --disable-decoder=flv1_bm --disable-decoder=h263_bm --disable-decoder=cavs_bm --disable-decoder=avs_bm --disable-decoder=vp3_bm --disable-decoder=vp8_bm \
	--disable-decoder=h264_v4l2m2m \
	--disable-vaapi --disable-hwaccel=h263_vaapi --disable-hwaccel=h264_vaapi --disable-hwaccel=hevc_vaapi --disable-hwaccel=mjpeg_vaapi \
	--disable-hwaccel=mpeg2_vaapi --disable-hwaccel=mpeg4_vaapi --disable-hwaccel=vc1_vaapi --disable-hwaccel=vp8_vaapi \
	--disable-hwaccel=vp8_vaapi --disable-hwaccel=wmv3_vaapi \
	--extra-libs="-L${extra_lib_path}/lib -L${extra_bmvid_lib_path} -L${extra_bmlib_path}/lib/pcie_windows -lbmjpulite -lbmjpuapi -lbmion -lbmvpuapi -lbmvppapi -lbmvideo -lbmlib -lbmcv -lvpp_cmodel -lharfbuzz -lfreetype -lpng -lz -lbz2 -lgb28181_sip -lws2_32" \
	--extra-cflags="-I${extra_inc_path}/bmcv/include -I${extra_lib_path}/include -I${extra_inc_path}/video/encoder/bm_enc_api/inc -I${extra_inc_path}/video/provider/cnm/encoder/inc -I${extra_inc_path}/video/decoder/bm_dec_api/inc -I${extra_inc_path}/allocator/ion/inc -I${extra_inc_path}/vpp/bmvppapi/inc -I${extra_inc_path}/jpeg/driver/bmjpuapi/inc -I${extra_bmlib_path}/include -I${extra_lib_path}/include/gbclient -DBM1684 -DBM_PCIE_MODE"
else
  echo do not understand input param, please check!
  exit 1
fi
