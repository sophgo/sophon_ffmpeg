/*
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

#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/time.h"
#if defined(BM1684)
#include "libavutil/hwcontext_bmcodec.h"
#endif

#include "avcodec.h"
#include "decode.h"
#include "libavutil/thread.h"
#if defined(BM1684)
#include "libavcodec/hwaccels.h"
#include "libavcodec/hwconfig.h"
#include "libavcodec/bm_codec.h"
#endif
#include "codec_internal.h"
#include "libavutil/half2float.h"
#include <ctype.h>    /* toupper() */
#include <sys/time.h> /* timeval */

#include "libavutil/hwcontext_bmcodec.h"
#include "libavutil/hwcontext_internal.h"
#include "hwconfig.h"
#include "fftools/ffmpeg.h"
#define MAX_FRAME_IN_BU

static void bmcodec_uninit(AVCodecContext *s)
{

    av_log(NULL, AV_LOG_TRACE, "[%s, %d] enter\n", __func__, __LINE__);
    av_buffer_unref(&s->hw_frames_ctx);
    av_log(NULL, AV_LOG_TRACE, "[%s, %d] leave\n", __func__, __LINE__);
}


static int bmcodec_init(AVCodecContext *s)
{
    InputStream *ist = s->opaque;
    AVHWFramesContext *frames_ctx;
    AVBmCodecFramesContext *frames_hwctx;
    BmCodecFramesContext *priv;
    int ret;
    int width_align;
    int height_align;

    av_log(NULL, AV_LOG_TRACE, "[%s, %d] enter\n", __func__, __LINE__);



    av_buffer_unref(&s->hw_frames_ctx);
    s->hw_frames_ctx = av_hwframe_ctx_alloc(s->hw_device_ctx);

    if (!s->hw_frames_ctx)
        return AVERROR(ENOMEM);

    frames_ctx   = (AVHWFramesContext*)s->hw_frames_ctx->data;
    //frames_hwctx = frames_ctx->hwctx;

    if (s->codec_id == AV_CODEC_ID_MJPEG) {
        switch (s->sw_pix_fmt) {
            case AV_PIX_FMT_YUV420P:
            case AV_PIX_FMT_YUVJ420P:
            case AV_PIX_FMT_NV12:
            case AV_PIX_FMT_NV21:
                width_align = 16;
                height_align = 16;
                break;
            case AV_PIX_FMT_YUV440P:
            case AV_PIX_FMT_YUVJ440P:
                width_align = 8;
                height_align = 16;
                break;
            case AV_PIX_FMT_YUV422P:
            case AV_PIX_FMT_YUVJ422P:
            case AV_PIX_FMT_NV16:
                width_align = 16;
                height_align = 8;
                break;
            default:
                width_align = 8;
                height_align = 8;
                break;
        }
        frames_ctx->width             = FFALIGN(s->coded_width,   width_align);
        frames_ctx->height            = FFALIGN(s->coded_height, height_align);
    }
    else if (s->codec_id == AV_CODEC_ID_H264) {
        frames_ctx->width             = FFALIGN(s->coded_width,  32);
        frames_ctx->height            = FFALIGN(s->coded_height, 16);
    } else {
        frames_ctx->width             = FFALIGN(s->coded_width,  32);
        frames_ctx->height            = s->coded_height;
    }
    frames_ctx->format            = AV_PIX_FMT_BMCODEC;
    frames_ctx->sw_format         = s->sw_pix_fmt;
    frames_ctx->initial_pool_size = 0; // Dynamic allocation in internal decoder
    priv = (BmCodecFramesContext *)frames_ctx->internal->priv;


    ret = av_hwframe_ctx_init(s->hw_frames_ctx);

    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error initializing a bmcodec frame pool\n");
        return ret;
    }
    av_log(NULL, AV_LOG_TRACE, "[%s, %d] leave\n", __func__, __LINE__);

    return 0;
}
const AVHWAccel ff_bmcodec_hwaccel = {
    .name           = "bmcodec",
    .type           = AVMEDIA_TYPE_VIDEO,
    .pix_fmt        = AV_PIX_FMT_BMCODEC,
    .init           = bmcodec_init,
    .uninit         = bmcodec_uninit,
};

