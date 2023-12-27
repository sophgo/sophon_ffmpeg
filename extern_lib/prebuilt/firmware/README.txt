libx264.so is only for pcie x86_64 and pcie aarch64, and will be called and run on A53 CPU. 

In order to use libx264.so, environment variables "BMX264_LIBFILE" need to be set.
for example:
    export BMX264_LIBFILE=/opt/sophon/sophon-ffmpeg-latest/lib/firmware/libx264.so

Example of the use of bmx264:
    $ export BMX264_LIBFILE=/opt/sophon/sophon-ffmpeg-latest/lib/firmware/libx264.so
    $ export BMCPU_RAMBOOTFILE=/opt/sophon/sophon-rpc-x.x.x/firmware/ramboot_rootfs.itb
    $ export BMCPU_FIPFILE=/opt/sophon/sophon-rpc-x.x.x/firmware/fip.bin
    $ ffmpeg -hwaccel bmcodec -hwaccel_device 0 \
      -c:v h264_bm -i input.264 -c:v bmx264 -b:v 2M \
      -enc-params "aq-mode=0:b-adapt=0:bframes=1:rc-lookahead=0:ref=2:scenecut=0:subme=2:trellis=0:weightp=0" \
      -y out.264
