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

#include <stdlib.h>

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_internal.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "ffmpeg.h"

static int bmcodec_get_buffer(AVCodecContext *s, AVFrame *frame, int flags)
{
    InputStream *ist = s->opaque;

    return av_hwframe_get_buffer(ist->hw_frames_ctx, frame, 0);
}

static void bmcodec_uninit(AVCodecContext *s)
{
    InputStream *ist = s->opaque;

    av_log(NULL, AV_LOG_TRACE, "[%s, %d] enter\n", __func__, __LINE__);
    av_buffer_unref(&ist->hw_frames_ctx);
    av_log(NULL, AV_LOG_TRACE, "[%s, %d] leave\n", __func__, __LINE__);
}


int bmcodec_init(AVCodecContext *s)
{
    InputStream *ist = s->opaque;
    AVHWFramesContext *frames_ctx;
    //AVBmCodecFramesContext *frames_hwctx;
    BmCodecFramesContext *priv;
    int ret;

    av_log(NULL, AV_LOG_TRACE, "[%s, %d] enter\n", __func__, __LINE__);

    if (!hw_device_ctx) {
        ret = av_hwdevice_ctx_create(&hw_device_ctx, AV_HWDEVICE_TYPE_BMCODEC,
                                     ist->hwaccel_device, NULL, 0);
        if (ret < 0)
            return ret;
    }

    av_buffer_unref(&ist->hw_frames_ctx);
    ist->hw_frames_ctx = av_hwframe_ctx_alloc(hw_device_ctx);
    if (!ist->hw_frames_ctx)
        return AVERROR(ENOMEM);

    frames_ctx   = (AVHWFramesContext*)ist->hw_frames_ctx->data;
    //frames_hwctx = frames_ctx->hwctx;

    frames_ctx->width             = FFALIGN(s->coded_width,  32);
    if (s->codec_id == AV_CODEC_ID_MJPEG) {
        frames_ctx->width             = FFALIGN(s->coded_width,  64);
        frames_ctx->height            = FFALIGN(s->coded_height, 16);
    }
    else
        frames_ctx->height            = FFALIGN(s->coded_height, 16);
    frames_ctx->format            = AV_PIX_FMT_BMCODEC;
    frames_ctx->sw_format         = s->sw_pix_fmt;
    frames_ctx->initial_pool_size = 0; // Dynamic allocation in internal decoder
    priv = frames_ctx->internal->priv;
    priv->codec_id = s->codec_id;

    ret = av_hwframe_ctx_init(ist->hw_frames_ctx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error initializing a bmcodec frame pool\n");
        return ret;
    }

    ist->hwaccel_get_buffer = bmcodec_get_buffer;
    ist->hwaccel_uninit     = bmcodec_uninit;

    av_log(NULL, AV_LOG_TRACE, "[%s, %d] leave\n", __func__, __LINE__);

    return 0;
}

