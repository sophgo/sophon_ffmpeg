/*
 * H.264 encoding using the bmx264 hardware acceleration
 *
 * Copyright (C) 2018-2020 Solan Shang <shulin.shang@bitmain.com>
 * Copyright (C) 2005  Mans Rullgard <mans@mansr.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */


#if defined(BM_PCIE_MODE) && defined(BM1684)

#include "config_components.h"
#include <strings.h>
#include <ctype.h>
#include <sys/time.h>

#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/eval.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/stereo3d.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavcodec/encode.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/decode.h"
#include "libavcodec/hwaccels.h"
#include "libavcodec/bm_codec.h"
#include "libavcodec/packet_internal.h"
#include "codec_internal.h"
#include "internal.h"


#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bmx264_common.h"

static const char* bmx264_preset_names[] = {
    "ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow", "slower", "veryslow", "placebo", 0
};
static const char* bmx264_tune_names[] = {
    "film", "animation", "grain", "stillimage", "psnr", "ssim", "fastdecode", "zerolatency", 0
};


static int encode_nals(AVCodecContext *ctx, AVPacket *pkt,
                       const x264_nal_t *nals, int nnal)
{
    BMX264Context *bmx = ctx->priv_data;
    uint8_t *src;
    uint8_t *dst;
    int i, size = bmx->sei_size, ret;

    if (!nnal)
        return 0;

    for (i = 0; i < nnal; i++)
        size += nals[i].i_payload;

    ret = ff_alloc_packet(ctx, pkt, size);
    if (ret < 0)
        return ret;

    dst = pkt->data;

    /* Write the SEI as part of the first frame. */
    if (bmx->sei_size > 0 && nnal > 0) {
        if (bmx->sei_size > size) {
            av_log(ctx, AV_LOG_ERROR, "Error: nal buffer is too small\n");
            return -1;
        }
        memcpy(dst, bmx->sei, bmx->sei_size);
        dst += bmx->sei_size;
        bmx->sei_size = 0;
        av_freep(&bmx->sei);
    }

    src = bmx->np_s.p_payload;
    for (i = 0; i < nnal; i++){
        memcpy(dst, src, nals[i].i_payload);
        src += nals[i].i_payload;
        dst += nals[i].i_payload;
    }

    return 1;
}

static int avfmt2_num_planes(int avfmt)
{
    switch (avfmt) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
    case AV_PIX_FMT_YUV444P:
        return 3;

    case AV_PIX_FMT_BGR0:
    case AV_PIX_FMT_BGR24:
    case AV_PIX_FMT_RGB24:
    case AV_PIX_FMT_GRAY8:
        return 1;

    default:
        return 3;
    }
}

static int reconfig_encoder(AVCodecContext *ctx)
{
    BMX264Context *bmx = ctx->priv_data;
    int b_reconfig = 0;

    if (bmx->params.vui.i_sar_height*ctx->sample_aspect_ratio.num !=
        ctx->sample_aspect_ratio.den * bmx->params.vui.i_sar_width) {
        bmx->params.vui.i_sar_height = ctx->sample_aspect_ratio.den;
        bmx->params.vui.i_sar_width  = ctx->sample_aspect_ratio.num;
        b_reconfig = 1;
    }

    if (bmx->params.rc.i_vbv_buffer_size != ctx->rc_buffer_size / 1000 ||
        bmx->params.rc.i_vbv_max_bitrate != ctx->rc_max_rate    / 1000) {
        bmx->params.rc.i_vbv_buffer_size = ctx->rc_buffer_size / 1000;
        bmx->params.rc.i_vbv_max_bitrate = ctx->rc_max_rate    / 1000;
        b_reconfig = 1;
    }

    if (bmx->params.rc.i_rc_method == X264_RC_ABR &&
        bmx->params.rc.i_bitrate != ctx->bit_rate / 1000) {
        bmx->params.rc.i_bitrate = ctx->bit_rate / 1000;
        b_reconfig = 1;
    }

    if (b_reconfig) {
        bmx264_msg_header_t header = {0};
        bmx264_msg_par_t    par[2] = {0};
        long ret64;

        /* ret = x264_encoder_reconfig(bmx->enc, &bmx->params); */

        /* 1. Transfer params data from host to device */
        int ret = bm_memcpy_s2d(bmx->device_handle, bmx->params_d, &(bmx->params));
        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR,  "Failed to copy x264 parameter from device to host\n");
            return AVERROR_EXTERNAL;
        }

        /* 2. Execute x264_encoder_reconfig() on device */
        strcpy(header.func, "x264_encoder_reconfig");
        header.par_num = 2;

        par[0].type = 0;
        sprintf(par[0].name, "%p", (void*)(bmx->enc));
        par[0].size = strlen(par[0].name);

        par[1].type = 0;
        sprintf(par[1].name, "%p", bmx->params_d_va);
        par[1].size = strlen(par[1].name);

        ret64 = bmx264_dispatch_task2(ctx, &header, par);
        if (ret64<0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Failed to run x264_encoder_reconfig() on bmx264_dispatch_task2()\n");
            return AVERROR_EXTERNAL;
        }
    }

    return 0;
}

static int BMX264_frame(AVCodecContext *ctx, AVPacket *pkt, const AVFrame *frame, int *got_packet)
{
    BMX264Context *bmx = ctx->priv_data;
    int output_nal = 0;
    int nnal, i, ret;
    int pict_type;

    struct timeval ps;
    if (bmx->perf) {
        gettimeofday(&ps, NULL);
    }

    /* Init pic_s, and transfer pic_s to pic_d */
    memset(&(bmx->pic_s), 0, sizeof(x264_picture_t));

    bmx->pic_s.img.i_csp   = bmx->params.i_csp;
    bmx->pic_s.img.i_plane = avfmt2_num_planes(ctx->sw_pix_fmt);

    if (frame) {
        AVHWFramesContext* frames_ctx = NULL;
        AVBmCodecFrame* hwpic;
        bm_device_mem_t planes_vpu_d = {0};
        int sw_format;
        char buf[128];

        if (frame->format != ctx->pix_fmt) {
            av_log(ctx, AV_LOG_ERROR,
                   "The frame pixel format(%s) is different from that(%s) of encoder context.\n",
                   av_get_pix_fmt_name(frame->format),
                   av_get_pix_fmt_name(ctx->pix_fmt));
            return AVERROR_EXTERNAL;
        }

        if (frame->hw_frames_ctx==NULL || frame->hw_frames_ctx->data==NULL) {
            av_log(ctx, AV_LOG_ERROR, "ERROR! Invalid frame hw_frames_ctx(NULL)!\n");
            return AVERROR_EXTERNAL;
        }

        frames_ctx = (AVHWFramesContext*)frame->hw_frames_ctx->data;
        if (frame->format != AV_PIX_FMT_BMCODEC) {
            av_log(ctx, AV_LOG_ERROR,
                   "The frame pixel hw format(%s) is not \'%s\'\n",
                   av_get_pix_fmt_name(frame->format),
                   av_get_pix_fmt_name(AV_PIX_FMT_BMCODEC));
            return AVERROR_EXTERNAL;
        }
        sw_format = frames_ctx->sw_format;
        if (sw_format != ctx->sw_pix_fmt) {
            av_log(ctx, AV_LOG_ERROR,
                   "The frame pixel sw format(%s) is different from that(%s) of encoder context.\n",
                   av_get_pix_fmt_name(sw_format),
                   av_get_pix_fmt_name(ctx->sw_pix_fmt));
            return AVERROR_EXTERNAL;
        }

        if (sw_format != AV_PIX_FMT_YUV420P && sw_format != AV_PIX_FMT_NV12) {
            snprintf(buf, sizeof(buf), "%d", sw_format);
            av_log(ctx, AV_LOG_ERROR,
                   "B. Specified pixel format %s is not supported\n",
                   (char *)av_x_if_null(av_get_pix_fmt_name(sw_format), buf));
            return AVERROR_PATCHWELCOME;
        }

        hwpic = (AVBmCodecFrame*)frame->data[4];
        if (hwpic && hwpic->maptype == 1) {
            av_log(ctx, AV_LOG_ERROR,
                   "For now, video encoder does NOT support compressed frame data!\n"
                   "Please use scale_bm to decompresse the data to uncompressed data first!\n");
            return AVERROR_EXTERNAL;
        }

        av_log(ctx, AV_LOG_TRACE, "The input pixel format is %s(%s)!\n",
               av_get_pix_fmt_name(frame->format), av_get_pix_fmt_name(sw_format));

        av_log(ctx, AV_LOG_TRACE, "data 0: %p\n", frame->data[0]);
        av_log(ctx, AV_LOG_TRACE, "data 1: %p\n", frame->data[1]);
        av_log(ctx, AV_LOG_TRACE, "data 2: %p\n", frame->data[2]);

        av_log(ctx, AV_LOG_TRACE, "line size 0: %d\n", frame->linesize[0]);
        av_log(ctx, AV_LOG_TRACE, "line size 1: %d\n", frame->linesize[1]);
        av_log(ctx, AV_LOG_TRACE, "line size 2: %d\n", frame->linesize[2]);

        for (i=0; i<((sw_format==AV_PIX_FMT_YUV420P)?3:2); i++) {
            if (frame->data[i] == NULL) {
                av_log(ctx, AV_LOG_ERROR, "ERROR! Invalid frame data[%d](%p)!\n", i, frame->data[i]);
                return AVERROR_EXTERNAL;
            }
            if (frame->linesize[i] <= 0) {
                av_log(ctx, AV_LOG_ERROR, "ERROR! Invalid frame linesize[%d](%d]!\n", i, frame->linesize[i]);
                return AVERROR_EXTERNAL;
            }
        }

        //------------------------------
        if (bmx->plane_d_va[0]==NULL) {
            int y_size, c_size;
            y_size = frame->data[1] - frame->data[0];
            if (sw_format == AV_PIX_FMT_NV12)
                c_size = y_size/2; // TODO
            else
                c_size = (frame->data[2] - frame->data[1]);

            ret = bmx264_pic_alloc(ctx, y_size, c_size, sw_format);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "Failed to bmx264_pic_alloc()\n");
                return AVERROR_EXTERNAL;
            }
        }

        for (i = 0; i < bmx->pic_s.img.i_plane; i++) {
            bmx->pic_s.img.plane[i]    = bmx->plane_d_va[i];
            bmx->pic_s.img.i_stride[i] = frame->linesize[i];
        }

        /* Transfer data from VPU memory to the memory accessed by A53 */
        planes_vpu_d.u.device.device_addr = (unsigned long)(frame->data[0]);
        planes_vpu_d.size = bmx->plane_d.size;
        ret = bm_memcpy_d2d_byte(bmx->device_handle, bmx->plane_d, 0,
                                 planes_vpu_d, 0, bmx->plane_d.size);
        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to copy x264 plane data from host to device\n");
            return AVERROR_EXTERNAL;
        }

        bmx->pic_s.i_pts = frame->pts;

        switch (frame->pict_type) {
        case AV_PICTURE_TYPE_I:
            bmx->pic_s.i_type = bmx->forced_idr > 0 ? X264_TYPE_IDR : X264_TYPE_KEYFRAME;
            break;
        case AV_PICTURE_TYPE_P:
            bmx->pic_s.i_type = X264_TYPE_P;
            break;
        case AV_PICTURE_TYPE_B:
            bmx->pic_s.i_type = X264_TYPE_B;
            break;
        default:
            bmx->pic_s.i_type = X264_TYPE_AUTO;
            break;
        }

        ret = reconfig_encoder(ctx);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to reconfig_encoder()\n");
            return AVERROR_EXTERNAL;
        }
    }

    ret = bm_memcpy_s2d(bmx->device_handle, bmx->pic_d, &bmx->pic_s);
    if (ret != 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to copy x264 picture from host to device\n");
        return AVERROR_EXTERNAL;
    }

    while (1) {
        long ret64;

        /* ret = x264_encoder_encode(bmx->enc, &nal, &nnal, frame? &bmx->pic_s: NULL, &pic_out); */
        bmx264_msg_header_t header = {0};
        bmx264_msg_par_t    par[4] = {0};

        strcpy(header.func, "x264_encoder_encode");
        header.par_num = 4;

        par[0].type = 0;
        sprintf(par[0].name, "%p", (void*)(bmx->enc));
        par[0].size = strlen(par[0].name)+1;

        par[1].type = 0;
        sprintf(par[1].name, "%p", bmx->nal_d_va);
        par[1].size = strlen(par[1].name)+1;

        par[2].type = 0;
        if (frame)
            sprintf(par[2].name, "%p", bmx->pic_d_va);
        else
            strcpy(par[2].name, "0");
        par[2].size = strlen(par[2].name)+1;

        par[3].type = 0;
        if (bmx->pic_out_d_va)
            sprintf(par[3].name, "%p", bmx->pic_out_d_va);
        else
            strcpy(par[3].name, "0");
        par[3].size = strlen(par[3].name)+1;

        ret64 = bmx264_dispatch_task2(ctx, &header, par);
        if (ret64<0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to exec x264_encoder_encode. ret=%ld\n", ret64);
            return AVERROR_EXTERNAL;
        }

        if (ret64>0) { /* frame size */
            ret = bm_memcpy_d2s(bmx->device_handle, bmx->nal_s, bmx->nal_d);
            if (ret != 0) {
                av_log(ctx, AV_LOG_ERROR, "Failed to copy nal data from device to host\n");
                return AVERROR_EXTERNAL;
            }

            nnal = *(bmx->np_s.p_nnal);
            output_nal = encode_nals(ctx, pkt, bmx->np_s.p_nal, nnal);
            if (output_nal < 0)
                return output_nal;
            break;
        }

        if (frame)
            break;

        /* ret = x264_encoder_delayed_frames(bmx->enc); */
        ret64 = bmx264_dispatch_task(ctx, "x264_encoder_delayed_frames", (void*)(bmx->enc));
        if (ret64 < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to run x264_encoder_delayed_frames()\n");
            return AVERROR_EXTERNAL;
        }
        if (ret64 == 0)
            break;
    }

    memset(&bmx->pic_out_s, 0, sizeof(x264_picture_t));

    ret = bm_memcpy_d2s(bmx->device_handle, &bmx->pic_out_s, bmx->pic_out_d);
    if (ret != 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to copy x264 picture out from device to host\n");
        return AVERROR_EXTERNAL;
    }
    pkt->pts = bmx->pic_out_s.i_pts;
    pkt->dts = bmx->pic_out_s.i_dts;


    switch (bmx->pic_out_s.i_type) {
    case X264_TYPE_IDR:
    case X264_TYPE_I:
        pict_type = AV_PICTURE_TYPE_I;
        break;
    case X264_TYPE_P:
        pict_type = AV_PICTURE_TYPE_P;
        break;
    case X264_TYPE_B:
    case X264_TYPE_BREF:
        pict_type = AV_PICTURE_TYPE_B;
        break;
    default:
        pict_type = AV_PICTURE_TYPE_NONE;
    }
#if FF_API_CODED_FRAME
    FF_DISABLE_DEPRECATION_WARNINGS
    ctx->coded_frame->pict_type = pict_type;
    FF_ENABLE_DEPRECATION_WARNINGS
#endif

    pkt->flags |= AV_PKT_FLAG_KEY*bmx->pic_out_s.b_keyframe;
    if (output_nal) {
        ff_side_data_set_encoder_stats(pkt, (bmx->pic_out_s.i_qpplus1 - 1) * FF_QP2LAMBDA, NULL, 0, pict_type);

#if FF_API_CODED_FRAME
        FF_DISABLE_DEPRECATION_WARNINGS
        ctx->coded_frame->quality = (bmx->pic_out_s.i_qpplus1 - 1) * FF_QP2LAMBDA;
        FF_ENABLE_DEPRECATION_WARNINGS
#endif
    }

    *got_packet = output_nal;

    if (bmx->perf) {
        struct timeval pe;
        gettimeofday(&pe, NULL);
        bmx->total_time += ((pe.tv_sec*1000.0 + pe.tv_usec/1000.0) -
                            (ps.tv_sec*1000.0 + ps.tv_usec/1000.0));
        if (output_nal)
            bmx->total_frame++;
    }

    return 0;
}

static av_cold int BMX264_close(AVCodecContext *avctx)
{
    BMX264Context *bmx = avctx->priv_data;
    int ret = 0;

    if (bmx->perf && bmx->total_time > 0.0f) {
        av_log(avctx, AV_LOG_INFO, "Frames encoded: %ld. Encoding speed: %.1ffps. Time used: %.3fsec.\n",
               bmx->total_frame, bmx->total_frame*1000/bmx->total_time, bmx->total_time/1000.0f);
    }
    av_freep(&avctx->extradata);
    av_freep(&bmx->sei);

    bmx264_pic_free(avctx);
    ret = bmx264_deinit(avctx);

    return ret;
}

static int convert_pix_fmt(enum AVPixelFormat pix_fmt)
{
    switch (pix_fmt) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P: return X264_CSP_I420;
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUVJ422P: return X264_CSP_I422;
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUVJ444P: return X264_CSP_I444;
    case AV_PIX_FMT_NV12:     return X264_CSP_NV12;
    case AV_PIX_FMT_NV16:
    case AV_PIX_FMT_NV20:     return X264_CSP_NV16;
    case AV_PIX_FMT_NV21:     return X264_CSP_NV21;
    case AV_PIX_FMT_GRAY8:    return X264_CSP_I400;
    };
    return 0;
}

static av_cold int BMX264_init(AVCodecContext *avctx)
{
    BMX264Context *bmx = avctx->priv_data;
    AVCPBProperties *cpb_props;
    AVBmCodecDeviceContext *bmcodec_device_hwctx = NULL;
    int sw,sh;
    int ret;
    long ret64;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    ret = bmcodec_get_device_hwctx(avctx, &bmcodec_device_hwctx);
    if (ret < 0)
        return ret;

    if (bmcodec_device_hwctx==NULL) {
        av_log(avctx, AV_LOG_ERROR, "bmcodec_device_hwctx is NULL\n");
        return AVERROR_EXTERNAL;
    }

    av_log(avctx, AV_LOG_DEBUG, "bmcodec_device_hwctx: device_idx=%d\n", bmcodec_device_hwctx->device_idx);

    ret = bmx264_init(avctx, bmcodec_device_hwctx->device_idx, avctx->width, avctx->height);
    if (ret<0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to bmx264_init()\n");
        return AVERROR(EINVAL);
    }

    if (bmx->preset || bmx->tune) {
        /* ret = x264_param_default_preset(&bmx->params, bmx->preset, bmx->tune); */
        bmx264_msg_header_t header = {0};
        bmx264_msg_par_t par[3] = {0};

        strcpy(header.func, "x264_param_default_preset");
        header.par_num = 3;

        par[0].type = 0;
        sprintf(par[0].name, "%p", bmx->params_d_va);
        par[0].size = strlen(par[0].name) + 1;

        par[1].type = 1;
        if (bmx->preset)
            sprintf(par[1].name, "%s", bmx->preset);
        par[1].size = strlen(par[1].name) + 1;

        par[2].type = 1;
        if (bmx->tune)
            sprintf(par[2].name, "%s", bmx->tune);
        par[2].size = strlen(par[2].name) + 1;

        ret64 = bmx264_dispatch_task2(avctx, &header, par);
        if (ret64<0) {
            int i;
            av_log(avctx, AV_LOG_ERROR, "Error setting preset/tune %s/%s. ret=%ld\n",
                   bmx->preset, bmx->tune, ret64);
            av_log(avctx, AV_LOG_INFO, "Possible presets:");
            for (i = 0; bmx264_preset_names[i]; i++)
                av_log(avctx, AV_LOG_INFO, " %s", bmx264_preset_names[i]);
            av_log(avctx, AV_LOG_INFO, "\n");
            av_log(avctx, AV_LOG_INFO, "Possible tunes:");
            for (i = 0; bmx264_tune_names[i]; i++)
                av_log(avctx, AV_LOG_INFO, " %s", bmx264_tune_names[i]);
            av_log(avctx, AV_LOG_INFO, "\n");
            return AVERROR(EINVAL);
        }
    } else {
        /*x264_param_default(&bmx->params);*/
        ret64 = bmx264_dispatch_task(avctx, "x264_param_default", bmx->params_d_va);
        if (ret64 < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to run x264_param_default() on bmx264_dispatch_task()\n");
            return AVERROR_EXTERNAL;
        }
    }

    /* download data from a53 to host */
    memset(&(bmx->params), 0, sizeof(bmx->params));
    ret = bm_memcpy_d2s(bmx->device_handle, &(bmx->params), bmx->params_d);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR,  "Failed to copy x264 parameter from device to host\n");
        return AVERROR(EINVAL);
    }

    bmx->params.i_log_level   = X264_LOG_NONE;
    bmx->params.i_csp         = convert_pix_fmt(avctx->sw_pix_fmt);
    bmx->params.i_bitdepth    = av_pix_fmt_desc_get(avctx->sw_pix_fmt)->comp[0].depth;

    if (avctx->bit_rate) {
        bmx->params.rc.i_bitrate   = avctx->bit_rate / 1000;
        bmx->params.rc.i_rc_method = X264_RC_ABR;
    }
    bmx->params.rc.i_vbv_buffer_size = avctx->rc_buffer_size / 1000;
    bmx->params.rc.i_vbv_max_bitrate = avctx->rc_max_rate    / 1000;
    if (avctx->rc_buffer_size && avctx->rc_initial_buffer_occupancy > 0 &&
        (avctx->rc_initial_buffer_occupancy <= avctx->rc_buffer_size)) {
        bmx->params.rc.f_vbv_buffer_init = (float)avctx->rc_initial_buffer_occupancy / avctx->rc_buffer_size;
    }

    if (avctx->gop_size >= 0)
        bmx->params.i_keyint_max = avctx->gop_size;

    bmx->params.i_width          = avctx->width;
    bmx->params.i_height         = avctx->height;
    av_reduce(&sw, &sh, avctx->sample_aspect_ratio.num, avctx->sample_aspect_ratio.den, 4096);
    bmx->params.vui.i_sar_width  = sw;
    bmx->params.vui.i_sar_height = sh;
    bmx->params.i_timebase_den = avctx->time_base.den;
    bmx->params.i_timebase_num = avctx->time_base.num;
    bmx->params.i_fps_num = avctx->time_base.den;
    bmx->params.i_fps_den = avctx->time_base.num * avctx->ticks_per_frame;

    bmx->params.analyse.b_psnr = avctx->flags & AV_CODEC_FLAG_PSNR;

    /* Only single thread is supported for now */
    //bmx->params.i_threads      = avctx->thread_count;

    bmx->params.b_open_gop     = !(avctx->flags & AV_CODEC_FLAG_CLOSED_GOP);

    bmx->params.i_slice_count  = avctx->slices;

    bmx->params.vui.b_fullrange = (avctx->sw_pix_fmt == AV_PIX_FMT_YUVJ420P ||
                                   avctx->sw_pix_fmt == AV_PIX_FMT_YUVJ422P ||
                                   avctx->sw_pix_fmt == AV_PIX_FMT_YUVJ444P ||
                                   avctx->color_range == AVCOL_RANGE_JPEG);

    if (avctx->colorspace != AVCOL_SPC_UNSPECIFIED)
        bmx->params.vui.i_colmatrix = avctx->colorspace;
    if (avctx->color_primaries != AVCOL_PRI_UNSPECIFIED)
        bmx->params.vui.i_colorprim = avctx->color_primaries;
    if (avctx->color_trc != AVCOL_TRC_UNSPECIFIED)
        bmx->params.vui.i_transfer  = avctx->color_trc;

    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER)
        bmx->params.b_repeat_headers = 0;

    if (bmx->bmx264_params) {
        AVDictionary *dict    = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, bmx->bmx264_params, "=", ":", 0)) {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                if (bmx264_param_parse(&bmx->params, en->key, en->value) < 0)
                    av_log(avctx, AV_LOG_WARNING,
                           "Error parsing option '%s = %s'.\n",
                           en->key, en->value);
            }

            av_dict_free(&dict);
        }
    }

    /* Update AVCodecContext with x264 parameters */
    avctx->has_b_frames = bmx->params.i_bframe ?  bmx->params.i_bframe_pyramid ? 2 : 1 : 0;
    if (avctx->max_b_frames < 0)
        avctx->max_b_frames = 0;

    avctx->bit_rate = bmx->params.rc.i_bitrate*1000;

    {
        char* p = bmx264_param2string(&bmx->params, 1);
        av_log(avctx, AV_LOG_INFO, "options: %s\n", p);
        free(p);
    }

    /* Upload data from host to a53 */
    ret = bm_memcpy_s2d(bmx->device_handle, bmx->params_d, &(bmx->params));
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR,  "Failed to copy x264 parameter from device to host\n");
        return AVERROR_EXTERNAL;
    }

    /* Open x264 encoder */
    /* bmx->enc = x264_encoder_open(&bmx->params); */
    bmx->enc = bmx264_dispatch_task(avctx, "x264_encoder_open", bmx->params_d_va);
    if (bmx->enc <= 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to run x264_encoder_open() on bmx264_dispatch_task(). ret=0x%lx\n",
               bmx->enc);
        return AVERROR_EXTERNAL;
    }

    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) {
        uint8_t *src;
        uint8_t *dst;
        int nnal;
        int s, i;

        /*s = x264_encoder_headers(bmx->enc, &nal, &nnal); */

        bmx264_msg_header_t header = {0};
        bmx264_msg_par_t    par[2] = {0};

        strcpy(header.func, "x264_encoder_headers");
        header.par_num = 2;

        par[0].type = 0;
        sprintf(par[0].name, "%p", (void*)(bmx->enc));
        par[0].size = strlen(par[0].name)+1;

        par[1].type = 0;
        sprintf(par[1].name, "%p", bmx->nal_d_va);
        par[1].size = strlen(par[1].name)+1;

        ret64 = bmx264_dispatch_task2(avctx, &header, par);
        if (ret64<0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to exec x264_encoder_headers\n");
            return AVERROR_EXTERNAL;
        }

        ret = bm_memcpy_d2s(bmx->device_handle, bmx->nal_s, bmx->nal_d);
        if (ret != 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to copy nals from device to host\n");
            return AVERROR_EXTERNAL;
        }

        dst = av_mallocz(s + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!dst) {
            return AVERROR(ENOMEM);
        }

        avctx->extradata = dst;

        nnal = *(bmx->np_s.p_nnal);
        src = bmx->np_s.p_payload;
        for (i = 0; i < nnal; i++) {
            int i_payload = bmx->np_s.p_nal[i].i_payload;
            /* Don't put the SEI in extradata. */
            if (bmx->np_s.p_nal[i].i_type == NAL_SEI) {
                av_log(avctx, AV_LOG_INFO, "%s\n", src+25);
                bmx->sei_size = i_payload;
                bmx->sei      = av_malloc(bmx->sei_size);
                if (!bmx->sei)
                    return AVERROR(ENOMEM);
                memcpy(bmx->sei, src, i_payload);
                continue;
            }
            memcpy(dst, src, i_payload);
            src += i_payload;
            dst += i_payload;
        }
        avctx->extradata_size = dst - avctx->extradata;
    }

    //syj--
    //cpb_props = ff_add_cpb_side_data(avctx);
    //if (!cpb_props)
    //    return AVERROR(ENOMEM);
    cpb_props->buffer_size = bmx->params.rc.i_vbv_buffer_size * 1000;
    cpb_props->max_bitrate = bmx->params.rc.i_vbv_max_bitrate * 1000;
    cpb_props->avg_bitrate = bmx->params.rc.i_bitrate         * 1000;

    if (bmx->perf) {
        bmx->total_time  = 0.0f;
        bmx->total_frame = 0L;
    }

    return 0;
}

static const enum AVPixelFormat pix_fmts_all[] = {
    AV_PIX_FMT_BMCODEC,
#if 0
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_GRAY8,

    AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_NV16,
    AV_PIX_FMT_NV21,
#endif
    AV_PIX_FMT_NONE
};

static av_cold void BMX264_init_static(AVCodec *codec)
{
    codec->pix_fmts = pix_fmts_all;
}

#define OFFSET(x) offsetof(BMX264Context, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "preset",        "Set the encoding preset (cf. x264 --fullhelp)",   OFFSET(preset),        AV_OPT_TYPE_STRING, { .str = "medium" }, 0, 0, VE},
    { "tune",          "Tune the encoding params (cf. x264 --fullhelp)",  OFFSET(tune),          AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE},
    { "forced-idr",   "If forcing keyframes, force them as IDR frames.",                                  OFFSET(forced_idr),  AV_OPT_TYPE_BOOL,   { .i64 = 0 }, -1, 1, VE },
    { "enc-params",  "Override the configuration using a :-separated list of key=value parameters", OFFSET(bmx264_params), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
    { "perf", "flag to indicate if do the performance testing", OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, VE },
    { NULL },
};

// static const AVCodecDefault bmx264_defaults[] = {
//     { "b",                "0" },
//     { "bf",               "-1" },
//     { "flags2",           "0" },
//     { "g",                "-1" },
//     { "flags",            "+cgop" },
//     { NULL },
// }; --lht

 #if CONFIG_BMX264_ENCODER  
static const AVClass bmx264_class = {
    .class_name = "bmx264",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

FFCodec ff_bmx264_encoder = {
    .p.name             = "bmx264",
    CODEC_LONG_NAME("bmx264 H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
    .p.type             = AVMEDIA_TYPE_VIDEO,
    .p.id               = AV_CODEC_ID_H264,
    .priv_data_size     = sizeof(BMX264Context),
    .init               = BMX264_init,
    .close              = BMX264_close,
    FF_CODEC_ENCODE_CB(BMX264_frame),
    .p.priv_class       = &bmx264_class,
    .p.capabilities     = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_DELAY,
    .caps_internal      = FF_CODEC_CAP_INIT_CLEANUP,
    .init_static_data = BMX264_init_static,
    .p.pix_fmts         = pix_fmts_all,
};

#endif

#endif

