/*
 * BM JPEG/MJPEG encoder
 * Copyright (c) 2018-2020 Solan Shang <shulin.shang@bitmain.com>
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

/**
 * @file
 * BM JPEG/MJPEG encoder
 *
 * @author Solan Shang <shulin.shang@bitmain.com>
 *
 * Unimplemented:
 *   - XXX
 */

#include <sys/time.h>

#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/decode.h"
#if defined(BM1684)
#include "libavcodec/hwaccel.h"
#include "libavcodec/bm_codec.h"
#endif
#include "internal.h"

#include "bm_jpeg_common.h"
#include "bmlib_runtime.h"

#define FRAMEBUFFER_ALIGNMENT 16

typedef struct {
    const AVClass *avclass;
    AVCodecContext *avctx;

    BmJpuJPEGEncoder* jpeg_encoder;
    int    hw_accel;
    int    is_dma_buffer; /* indicate if the input frame buffer is DMA buffer */
    int    soc_idx;
    int    quality;       /* quality factor */

    int    perf;          /* indicate if profiling performance */

    double total_time;    /* in unit of ms */
    long   total_frame;
    bm_handle_t handle;
} BMJpegEncContext;

typedef struct {
    AVCodecContext *avctx;
    AVPacket* avpkt;
} bs_buffer_t;

static void* acquire_output_buffer(void *context, size_t size, void **acquired_handle)
{
    bs_buffer_t* s = (bs_buffer_t*)(context);
    void *mem;
    int ret;

    ret = ff_alloc_packet2(s->avctx, s->avpkt, size, size);
    if (ret < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Error! Failed ff_alloc_packet2()!\n");
        return NULL;
    }

    mem = (void*)(s->avpkt->data);
    *acquired_handle = mem;
    return mem;
}
static void finish_output_buffer(void *context, void *acquired_handle)
{
    ((void)(context));
}

static av_cold int bm_jpegenc_init(AVCodecContext *avctx)
{
    BMJpegEncContext* ctx = (BMJpegEncContext*)(avctx->priv_data);
#if defined(BM1684)
    AVBmCodecDeviceContext *bmcodec_device_hwctx = NULL;
#endif
    int raw_size = avctx->width * avctx->height * 3/2;
    /* assume the min compression ratio is 2 */
    int min_ratio = 2;
    int bs_buffer_size;
    int sw_pix_fmt;
    int ret;

    ctx->hw_accel = 0;
#if defined(BM1684)
    ret = bmcodec_get_device_hwctx(avctx, &bmcodec_device_hwctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "bmcodec_get_device_hwctx failed\n");
        return ret;
    }
    if (bmcodec_device_hwctx) {
        ctx->hw_accel = 1;
        ctx->soc_idx = bmcodec_device_hwctx->device_idx;
        av_log(avctx, AV_LOG_DEBUG, "bmcodec_device_hwctx: device_idx=%d\n", bmcodec_device_hwctx->device_idx);
    }
#endif

#ifndef BM_PCIE_MODE
    ctx->soc_idx = 0;
#endif

    ret = bm_dev_request(&ctx->handle, ctx->soc_idx);
    if(ret != BM_SUCCESS)
    {
        av_log(ctx, AV_LOG_ERROR, "bmlib create handle failed\n");
    }

    av_log(avctx, AV_LOG_INFO, "width   = %d\n", avctx->width);
    av_log(avctx, AV_LOG_INFO, "height  = %d\n", avctx->height);
    av_log(avctx, AV_LOG_INFO, "pix_fmt = %s\n", av_get_pix_fmt_name(avctx->pix_fmt));
    if (ctx->hw_accel)
        av_log(avctx, AV_LOG_INFO, "sw_pix_fmt = %s\n", av_get_pix_fmt_name(avctx->sw_pix_fmt));

    /* The size for YUV 420/422/444 */
    if (ctx->hw_accel)
        sw_pix_fmt = avctx->sw_pix_fmt;
    else
        sw_pix_fmt = avctx->pix_fmt;

    if (sw_pix_fmt == AV_PIX_FMT_YUV420P || sw_pix_fmt == AV_PIX_FMT_YUVJ420P ||
        sw_pix_fmt == AV_PIX_FMT_NV12 || sw_pix_fmt == AV_PIX_FMT_NV21)
        raw_size = avctx->width * avctx->height * 3/2;
    else if (sw_pix_fmt == AV_PIX_FMT_YUV422P || sw_pix_fmt == AV_PIX_FMT_YUVJ422P ||
        sw_pix_fmt == AV_PIX_FMT_NV16)
        raw_size = avctx->width * avctx->height * 2;
    else if (sw_pix_fmt == AV_PIX_FMT_YUV444P || sw_pix_fmt == AV_PIX_FMT_YUVJ444P)
        raw_size = avctx->width * avctx->height * 3;
    else /* if (sw_pix_fmt == AV_PIX_FMT_GRAY8) */
        raw_size = avctx->width * avctx->height;

    if(ctx->quality >= 95)
        min_ratio = 1;

#define BS_MASK (1024*16-1)
    bs_buffer_size = raw_size/min_ratio;
    /* bitstream buffer size in unit of 16k */
    bs_buffer_size = (bs_buffer_size+BS_MASK)&(~BS_MASK);
    if (bs_buffer_size >= 16*1023*1024)
        bs_buffer_size = 16*1023*1024;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    ctx->avctx = avctx; // TODO

    bmjpu_setup_logging();

    /* Load JPU */
    ret = bm_jpu_enc_load(ctx->soc_idx);
    if (ret != BM_JPU_ENC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "bm_jpu_enc_load failed!\n");
        return AVERROR_INVALIDDATA;
    }

    bm_jpu_jpeg_enc_open(&(ctx->jpeg_encoder), 0, bs_buffer_size, ctx->soc_idx);
    if (ret != BM_JPU_ENC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "bm_jpu_jpeg_enc_open failed!\n");
        return AVERROR_INVALIDDATA;
    }
    if (!ctx->jpeg_encoder) {
        av_log(avctx, AV_LOG_ERROR, "ctx->jpeg_encoder is NULL!\n");
        return AVERROR_INVALIDDATA;
    }

    if (ctx->perf) {
        ctx->total_time  = 0.0f;
        ctx->total_frame = 0L;
    }

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;
}

static av_cold int bm_jpegenc_close(AVCodecContext *avctx)
{
    BMJpegEncContext* ctx = (BMJpegEncContext *)(avctx->priv_data);
    int ret;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    if (ctx->perf) {
        if (ctx->total_time > 0.0f) {
            av_log(avctx, AV_LOG_INFO, "Frames encoded: %ld. Encoding speed: %.1ffps. Time used: %.3fsec.\n",
                   ctx->total_frame, ctx->total_frame*1000/ctx->total_time, ctx->total_time/1000.0f);
        }
    }

    if (ctx->jpeg_encoder) {
        bm_jpu_jpeg_enc_close(ctx->jpeg_encoder);
        ctx->jpeg_encoder = NULL;

        ret = bm_jpu_enc_unload(ctx->soc_idx);
        if (ret != BM_JPU_ENC_RETURN_CODE_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to call bm_jpu_enc_unload\n");
            return AVERROR_UNKNOWN;
        }
    }

    bm_dev_free(ctx->handle);
    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;
}

static int bm_jpegenc_encode_frame(AVCodecContext *avctx,
                                   AVPacket *avpkt,
                                   const AVFrame *frame,
                                   int *got_packet)
{
    BMJpegEncContext* ctx = (BMJpegEncContext *)(avctx->priv_data);
    BmJpuJPEGEncParams enc_params;
    BmJpuFramebuffer framebuffer;
    bm_device_mem_t wrapped_mem;
    BmJpuImageFormat out_image_format = BM_JPU_IMAGE_FORMAT_YUV420P;
    BmJpuEncReturnCodes enc_ret;
    bs_buffer_t bs_buffer;
    void *acquired_handle;
    size_t bs_length;
    int width, height, format;
    char buf[128];
    int i, ret = 0;
#ifdef BM_PCIE_MODE
    unsigned int size;
#endif
    struct timeval ps = {0};

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    *got_packet = 0;

    if (frame == NULL) {
        av_log(avctx, AV_LOG_ERROR, "ERROR! frame is null!\n");
        return AVERROR_EXTERNAL;
    }

    if (ctx->perf) {
        gettimeofday(&ps, NULL);
    }

    width  = frame->width;
    height = frame->height;

    // TODO
#if 0
    /* get data from upstream */
    AVFrame * pic;
    pic = av_frame_clone(frame);
    if (!pic)
        return AVERROR(ENOMEM);
#endif

    /* Check if input format is compatible with encoder */
    if (frame->format != avctx->pix_fmt &&
        !((frame->format == AV_PIX_FMT_YUVJ420P && avctx->pix_fmt == AV_PIX_FMT_YUV420P) ||
          (frame->format == AV_PIX_FMT_YUV420P && avctx->pix_fmt == AV_PIX_FMT_YUVJ420P) ||
          (frame->format == AV_PIX_FMT_YUVJ422P && avctx->pix_fmt == AV_PIX_FMT_YUV422P) ||
          (frame->format == AV_PIX_FMT_YUV422P && avctx->pix_fmt == AV_PIX_FMT_YUVJ422P) ||
          (frame->format == AV_PIX_FMT_YUVJ444P && avctx->pix_fmt == AV_PIX_FMT_YUV444P) ||
          (frame->format == AV_PIX_FMT_YUV444P && avctx->pix_fmt == AV_PIX_FMT_YUVJ444P))) {
        snprintf(buf, sizeof(buf), "%d", frame->format);
        av_log(avctx, AV_LOG_ERROR,
               "A. Specified pixel format %s is not supported\n",
               (char *)av_x_if_null(av_get_pix_fmt_name(frame->format), buf));
        return AVERROR_EXTERNAL;
    }
    if (ctx->hw_accel) {
        AVHWFramesContext* frm_ctx = (AVHWFramesContext*)frame->hw_frames_ctx->data;
        if (frame->format != AV_PIX_FMT_BMCODEC) {
            snprintf(buf, sizeof(buf), "%d", frame->format);
            av_log(avctx, AV_LOG_ERROR,
                   "Specified pixel hw format %s is not supported\n",
                   (char *)av_x_if_null(av_get_pix_fmt_name(frame->format), buf));
            return AVERROR_EXTERNAL;
        }
        format = frm_ctx->sw_format;
        if (format != avctx->sw_pix_fmt) {
            snprintf(buf, sizeof(buf), "%d", format);
            av_log(avctx, AV_LOG_ERROR,
                   "Specified pixel sw format %s is not supported\n",
                   (char *)av_x_if_null(av_get_pix_fmt_name(format), buf));
            return AVERROR_EXTERNAL;
        }
    } else {
        format = frame->format;
    }

    if (format == AV_PIX_FMT_YUV420P || format == AV_PIX_FMT_YUVJ420P) {
        out_image_format = BM_JPU_IMAGE_FORMAT_YUV420P;
    } else if (format == AV_PIX_FMT_NV12) {
        out_image_format = BM_JPU_IMAGE_FORMAT_NV12;
    } else if (format == AV_PIX_FMT_NV21) {
        out_image_format = BM_JPU_IMAGE_FORMAT_NV21;
    } else if (format == AV_PIX_FMT_YUV422P || format == AV_PIX_FMT_YUVJ422P) {
        out_image_format = BM_JPU_IMAGE_FORMAT_YUV422P;
    } else if (format == AV_PIX_FMT_NV16) {
        out_image_format = BM_JPU_IMAGE_FORMAT_NV16;
    } else if (format == AV_PIX_FMT_YUV444P || format == AV_PIX_FMT_YUVJ444P) {
        out_image_format = BM_JPU_IMAGE_FORMAT_YUV444P;
    } else if (format == AV_PIX_FMT_GRAY8) {
        if (frame->color_range == AVCOL_RANGE_MPEG) {
            av_log(avctx, AV_LOG_ERROR,
                   "For gray pixel format, bm jpeg encoder does not support the color range: %s.\n",
                   av_color_range_name(frame->color_range));
        }
        if (frame->color_range != AVCOL_RANGE_JPEG) {
            av_log(avctx, AV_LOG_WARNING,
                   "For gray pixel format, the input color range is (%s). Encode it as full color range.\n",
                   av_color_range_name(frame->color_range));
        }
        out_image_format = BM_JPU_IMAGE_FORMAT_GRAY;
    } else {
        snprintf(buf, sizeof(buf), "%d", format);
        av_log(avctx, AV_LOG_ERROR,
               "B. Specified pixel format %s is not supported\n",
               (char *)av_x_if_null(av_get_pix_fmt_name(format), buf));
        return AVERROR_PATCHWELCOME;
    }

    memset(&framebuffer, 0, sizeof(BmJpuFramebuffer));

    av_log(avctx, AV_LOG_DEBUG, "width =%d\n", width);
    av_log(avctx, AV_LOG_DEBUG, "height=%d\n", height);

    for (i=0; i<8; i++)
        av_log(avctx, AV_LOG_TRACE, "data %d: %p, line size %d: %d\n",
               i, frame->data[i], i, frame->linesize[i]);

    av_log(avctx, AV_LOG_DEBUG, "hw_accel=%d\n", ctx->hw_accel);
    if (ctx->hw_accel || ctx->is_dma_buffer) {
        //AVBmCodecFrame* hwpic = (AVBmCodecFrame*)frame->data[4];
        int y_size, c_size, total_size;
        int channel_idx = ctx->hw_accel ? 0 : 4;

        switch (format) {
            case AV_PIX_FMT_GRAY8:
                if (frame->data[channel_idx] == NULL) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame data!\n");
                    return AVERROR_EXTERNAL;
                }
                if (frame->linesize[channel_idx] <= 0) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame linesize!\n");
                    return AVERROR_EXTERNAL;
                }
                y_size = frame->linesize[channel_idx] * frame->height;
                c_size = 0;

                framebuffer.y_stride    = frame->linesize[channel_idx];
                framebuffer.y_offset    = 0;

                break;
            case AV_PIX_FMT_NV12:
            case AV_PIX_FMT_NV21:
            case AV_PIX_FMT_NV16:
                if (frame->data[channel_idx] == NULL || frame->data[channel_idx + 1] == NULL) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame data!\n");
                    return AVERROR_EXTERNAL;
                }
                if (frame->linesize[channel_idx] <= 0 || frame->linesize[channel_idx + 1] <= 0) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame linesize!\n");
                    return AVERROR_EXTERNAL;
                }
                y_size = frame->data[channel_idx + 1] - frame->data[channel_idx];
                c_size = frame->linesize[channel_idx + 1] * frame->height / 2;

                framebuffer.y_stride    = frame->linesize[channel_idx];
                framebuffer.cbcr_stride = frame->linesize[channel_idx + 1];
                framebuffer.y_offset    = 0;
                if (format == AV_PIX_FMT_NV21) {
                    framebuffer.cb_offset   = y_size + 1;
                    framebuffer.cr_offset   = y_size;
                } else {
                    framebuffer.cb_offset   = y_size;
                    framebuffer.cr_offset   = y_size + 1;
                }

                break;
            case AV_PIX_FMT_YUV420P:
            case AV_PIX_FMT_YUVJ420P:
            case AV_PIX_FMT_YUV422P:
            case AV_PIX_FMT_YUVJ422P:
            case AV_PIX_FMT_YUV444P:
            case AV_PIX_FMT_YUVJ444P:
            default:
                if (frame->data[channel_idx] == NULL ||
                    frame->data[channel_idx + 1] == NULL ||
                    frame->data[channel_idx + 2] == NULL) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame data!\n");
                    return AVERROR_EXTERNAL;
                }
                if (frame->linesize[channel_idx] <= 0 ||
                    frame->linesize[channel_idx + 1] <= 0 ||
                    frame->linesize[channel_idx + 2] <= 0) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame linesize!\n");
                    return AVERROR_EXTERNAL;
                }
                y_size = frame->data[channel_idx + 1] - frame->data[channel_idx];
                c_size = frame->data[channel_idx + 2] - frame->data[channel_idx + 1];

                framebuffer.y_stride    = frame->linesize[channel_idx];
                framebuffer.cbcr_stride = frame->linesize[channel_idx + 1];
                framebuffer.y_offset    = 0;
                framebuffer.cb_offset   = y_size;
                framebuffer.cr_offset   = y_size + c_size;

                break;
        }

        total_size = y_size;
        if (format != AV_PIX_FMT_GRAY8)
            total_size += c_size*2;

        wrapped_mem.flags.u.mem_type = BM_MEM_TYPE_DEVICE;
        wrapped_mem.u.device.device_addr = (unsigned long long)frame->data[channel_idx];
        wrapped_mem.u.device.dmabuf_fd = 1;
        wrapped_mem.size = total_size;

        framebuffer.dma_buffer = &wrapped_mem;
    } else {
        /* The input frames come in Non-DMA memory */
        uint8_t* p_virt_addr;
        uint8_t *src_y, *src_u, *src_v;
        uint8_t *dst_y, *dst_u, *dst_v;
        int src_stride_y, src_stride_u, src_stride_v;
        int dst_stride_y, dst_stride_u, dst_stride_v;
        BmJpuFramebufferSizes calculated_sizes;

        /* Initialize the input framebuffer */
        ret = bm_jpu_calc_framebuffer_sizes(width, height,
                                      FRAMEBUFFER_ALIGNMENT,
                                      out_image_format,
                                      &calculated_sizes);
        if (ret != BM_JPU_ENC_RETURN_CODE_OK) {
            av_log(avctx, AV_LOG_ERROR,
                   "Initialize the input framebuffer failed.\n");
            return AVERROR(ENOMEM);
        }

        av_log(avctx, AV_LOG_DEBUG, "\twidth =%d\n", width);
        av_log(avctx, AV_LOG_DEBUG, "\theight=%d\n", height);
        av_log(avctx, AV_LOG_DEBUG, "calculated_sizes:\n");
        av_log(avctx, AV_LOG_DEBUG,
               "\timage_format=%d\n", calculated_sizes.image_format);
        av_log(avctx, AV_LOG_DEBUG,
               "\taligned_frame_width =%d\n", calculated_sizes.aligned_frame_width);
        av_log(avctx, AV_LOG_DEBUG,
               "\taligned_frame_height=%d\n", calculated_sizes.aligned_frame_height);
        av_log(avctx, AV_LOG_DEBUG, "\ty_stride=%d\n", calculated_sizes.y_stride);
        av_log(avctx, AV_LOG_DEBUG, "\tcbcr_stride=%d\n", calculated_sizes.cbcr_stride);
        av_log(avctx, AV_LOG_DEBUG, "\ty_size=%d\n", calculated_sizes.y_size);
        av_log(avctx, AV_LOG_DEBUG, "\tcbcr_size=%d\n", calculated_sizes.cbcr_size);
        av_log(avctx, AV_LOG_DEBUG, "\ttotal_size=%d\n", calculated_sizes.total_size);

        framebuffer.y_stride    = calculated_sizes.y_stride;
        framebuffer.cbcr_stride = calculated_sizes.cbcr_stride;
        framebuffer.y_offset    = 0;
        framebuffer.cb_offset   = calculated_sizes.y_size;
        framebuffer.cr_offset   = calculated_sizes.y_size + calculated_sizes.cbcr_size;

        if (framebuffer.dma_buffer != NULL) {
            free (framebuffer.dma_buffer);
            framebuffer.dma_buffer = NULL;
        }
        framebuffer.dma_buffer = (bm_device_mem_t*)malloc(sizeof(bm_device_mem_t));
        ret = bm_malloc_device_byte(ctx->handle, framebuffer.dma_buffer, calculated_sizes.total_size);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "bm_malloc_device_byte failed.\n");
            return AVERROR(ENOMEM);
        }

        /* Load the input pixels into the DMA buffer */
#ifdef BM_PCIE_MODE
        size = bm_mem_get_device_size(*(framebuffer.dma_buffer));
        p_virt_addr = av_mallocz(size);
        if (!p_virt_addr) {
            av_log(avctx, AV_LOG_ERROR,
                   "could not allocate buffer for input data\n");
            return AVERROR(ENOMEM);
        }
#else
        ret = bm_mem_mmap_device_mem(ctx->handle, framebuffer.dma_buffer, (unsigned long long *)&p_virt_addr);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "bm_mem_mmap_device_mem failed!\n");
            return AVERROR(ENOMEM);
        }
#endif
        dst_y = p_virt_addr + framebuffer. y_offset;
        dst_u = p_virt_addr + framebuffer.cb_offset;
        dst_v = p_virt_addr + framebuffer.cr_offset;
        dst_stride_y = framebuffer.   y_stride;
        dst_stride_u = framebuffer.cbcr_stride;
        dst_stride_v = framebuffer.cbcr_stride;

        /* Copy YUV420P to YUV420P */
        src_y = frame->data[0];
        src_u = frame->data[1];
        src_v = frame->data[2];
        src_stride_y = frame->linesize[0];
        src_stride_u = frame->linesize[1];
        src_stride_v = frame->linesize[2];

        {
            uint8_t *dst_data[4] = {dst_y, dst_u, dst_v, NULL};
            int dst_linesizes[4] = {dst_stride_y, dst_stride_u, dst_stride_v, 0};
            const uint8_t *src_data[4] = {src_y, src_u, src_v, NULL};
            const int src_linesizes[4] = {src_stride_y, src_stride_u, src_stride_v, 0};

            av_image_copy(dst_data, dst_linesizes,
                          src_data, src_linesizes,
                          format, width, height);
        }
#ifdef BM_PCIE_MODE
        ret = bm_memcpy_s2d_partial(ctx->handle, *(framebuffer.dma_buffer), p_virt_addr, size);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "bm_memcpy_s2d_partial failed!\n");
            return AVERROR(ENOMEM);
        }
        av_free(p_virt_addr);
#else
        /* Flush cache to DMA buffer */
        ret = bm_mem_flush_device_mem(ctx->handle, framebuffer.dma_buffer);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "bm_mem_flush_partial_device_mem failed!\n");
            return AVERROR(ENOMEM);
        }
        /* Unmap the DMA buffer of the raw frame */
        ret = bm_mem_unmap_device_mem(ctx->handle, (void *)p_virt_addr, framebuffer.dma_buffer->size);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR,
                   "bm_mem_unmap_device_mem failed!\n");
            return AVERROR(ENOMEM);
        }
#endif
    }

    bs_buffer.avctx = avctx;
    bs_buffer.avpkt = avpkt;

    /* Set up the encoding parameters */
    memset(&enc_params, 0, sizeof(enc_params));
    enc_params.frame_width    = width;
    enc_params.frame_height   = height;
    enc_params.quality_factor = ctx->quality;
    enc_params.image_format   = out_image_format;
    enc_params.acquire_output_buffer = acquire_output_buffer;
    enc_params.finish_output_buffer  = finish_output_buffer;
    enc_params.output_buffer_context = (void*)(&bs_buffer);

    /* Do the actual encoding */
    enc_ret = bm_jpu_jpeg_enc_encode(ctx->jpeg_encoder,
                                     &framebuffer,
                                     &enc_params,
                                     &acquired_handle,
                                     &bs_length);

    /* The framebuffer's DMA buffer isn't needed anymore, since we just
     * did the encoding, so deallocate it */
    if (!ctx->is_dma_buffer) {
        bm_free_device(ctx->handle, *(framebuffer.dma_buffer));
        if (framebuffer.dma_buffer != NULL) {
            free(framebuffer.dma_buffer);
            framebuffer.dma_buffer = NULL;
        }
    }

    if (enc_ret != BM_JPU_ENC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "could not encode this image : %s\n",
               bm_jpu_enc_error_string(enc_ret));
        av_packet_unref(avpkt);
        return AVERROR_UNKNOWN;
    }

    if (ctx->perf) {
        struct timeval pe;
        gettimeofday(&pe, NULL);
        ctx->total_time += ((pe.tv_sec*1000.0 + pe.tv_usec/1000.0) -
                            (ps.tv_sec*1000.0 + ps.tv_usec/1000.0));
        ctx->total_frame++;
    }
    *got_packet = 1;

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return ret;
}

#define OFFSET(x) offsetof(BMJpegEncContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] = {
    { "is_dma_buffer", "flag to indicate if the input frame buffer is continuous physical buffer",
        OFFSET(is_dma_buffer), AV_OPT_TYPE_FLAGS, {.i64 = 1}, 0, 1, FLAGS },
    { "sophon_idx", "Sophon device index when running in pcie mode",
        OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, INT_MIN, INT_MAX, FLAGS },
    { "qf", "quality factor of jpeg encoder",
        OFFSET(quality), AV_OPT_TYPE_INT, {.i64 = 85}, 0, 100, FLAGS },
    { "perf", "flag to indicate if profiling performance",
        OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { NULL},
};

static const AVClass bm_jpegenc_class = {
    .class_name = "jpeg_bm_encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_jpeg_bm_encoder = {
    .name           = "jpeg_bm",
    .long_name      = NULL_IF_CONFIG_SMALL("BM JPEG ENCODER"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_MJPEG,
    .priv_data_size = sizeof(BMJpegEncContext),
    .init           = bm_jpegenc_init,
    .close          = bm_jpegenc_close,
    .encode2        = bm_jpegenc_encode_frame,
    .priv_class     = &bm_jpegenc_class,
    .capabilities   = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_INTRA_ONLY,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .pix_fmts       = (const enum AVPixelFormat[]){
        AV_PIX_FMT_BMCODEC,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_NV21,
        AV_PIX_FMT_NV16,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_NONE
    },
    .wrapper_name   = "bm"
};
