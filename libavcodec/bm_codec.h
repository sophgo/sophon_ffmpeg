/*
 * BM video codec
 * Copyright (c) 2020 Solan Shang <shulin.shang@bitmain.com>
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


#if defined(BM1684)
static int bmcodec_get_device_hwctx(AVCodecContext *avctx, AVBmCodecDeviceContext** p_device_hwctx)
{
    (*p_device_hwctx) = NULL;

    if (avctx->pix_fmt != AV_PIX_FMT_BMCODEC && avctx->hw_frames_ctx==NULL && avctx->hw_device_ctx==NULL)
        return 0;

    av_log(avctx, AV_LOG_DEBUG, "avctx: pix_fmt=%d, hw_frames_ctx=%p, hw_device_ctx=%p\n",
           avctx->pix_fmt, avctx->hw_frames_ctx, avctx->hw_device_ctx);

    if (avctx->hw_frames_ctx) {
        AVHWFramesContext* frames_ctx = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
        if (frames_ctx->format != AV_PIX_FMT_BMCODEC) {
            av_log(avctx, AV_LOG_ERROR, "[%s,%d] invalid pixel format\n", __func__, __LINE__);
            return AVERROR(EINVAL);
        }
        (*p_device_hwctx) = frames_ctx->device_ctx->hwctx;
    } else if (avctx->hw_device_ctx) {
        AVHWDeviceContext* hwdev_ctx = (AVHWDeviceContext*)avctx->hw_device_ctx->data;
        if (hwdev_ctx->type != AV_HWDEVICE_TYPE_BMCODEC) {
            av_log(avctx, AV_LOG_ERROR, "[%s,%d] invalid hw device type\n", __func__, __LINE__);
            return AVERROR(EINVAL);
        }
        (*p_device_hwctx) = hwdev_ctx->hwctx;
    } else {
        av_log(avctx, AV_LOG_ERROR, "[%s,%d] invalid hw device info\n", __func__, __LINE__);
        return AVERROR(EINVAL);
    }

    return 0;
}
#endif
