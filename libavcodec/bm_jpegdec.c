/*
 * BM JPEG/MJPEG decoder
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
 * BM JPEG/MJPEG decoder
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
#include "libavutil/frame.h"
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
#include "libavutil/hwcontext_bmcodec.h"
#endif

#include "libavcodec/avcodec.h"
#include "libavcodec/decode.h"
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
#include "libavcodec/hwaccels.h"
#include "libavcodec/bm_codec.h"
#include "libavcodec/hwconfig.h"
#endif
#include "codec_internal.h"

#include "bm_jpeg_interface.h"
#include "bm_jpeg_logging.h"
#include "bm_jpeg_common.h"
#include "bmlib_runtime.h"
#include "config_components.h"

#define HEAP_MASK_1_2 0x06

typedef struct {
    AVCodecContext* avctx; // TODO
    BmJpuJPEGDecoder *jpeg_decoder;
    int hw_accel;

    int bs_buffer_size; /* Kbytes */
    int chroma_interleave;

    /**
     * num_extra_framebuffers is used for instructing decoder to allocate this many
     * more framebuffers. Usually this value is zero, but in certain cases where many
     * JPEGs need to be decoded quickly, or the DMA buffers of decoded frames need to
     * be kept around elsewhere, having more framebuffers available can be helpful.
     * Note though that more framebuffers also means more DMA memory consumption.
     * For still jpeg, keep this to zero (at fault).
     * For motion jpeg, set this to 2 at least.
     */
    int num_extra_framebuffers;

    int soc_idx;
    int zero_copy;

    enum AVPixelFormat old_pix_fmt;

    int    perf;       /* indicate if do the performance testing */

    double total_time; // ms
    long   total_frame;
    bm_handle_t handle;

    int framebuffer_recycle;
    size_t framebuffer_size;
} BMJpegDecContext;

#ifndef MAGIC_JPEG
#define MAGIC_JPEG          0x4A504547
#endif
#ifndef MAGIC_MAT
#define MAGIC_MAT           0x204D4154
#endif
typedef struct {
    unsigned int magic_number; //0x4A504547 JPEG  0x204D4154 MAT
    BMJpegDecContext* ctx;
    BmJpuFramebuffer* framebuffer;
    bm_device_mem_t* dma_buffer_to_user;
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
    AVBmCodecFrame*   hwpic;
#endif
#ifdef BM_PCIE_MODE
    void *data;
#else
    uint8_t* vaddr;
#endif

    int release_count;
} bm_opaque_t;

static int head_functions_set_up = 0;

static enum AVPixelFormat pix_fmts[3] = {
    AV_PIX_FMT_BMCODEC,
    AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_NONE
};

static void* bmjpu_heap_alloc_func(size_t const size, void *context, char const *file, int const line, char const *fn)
{
    void *ptr = av_mallocz(size);
    return ptr;
}
static void bmjpu_heap_free_func(void *memblock, size_t const size, void *context, char const *file, int const line, char const *fn)
{
    av_free(memblock);
}
static void bmjpu_setup_heap_allocator_functions(void)
{
    av_log(NULL, AV_LOG_TRACE, "Enter %s\n", __func__);
    if (!head_functions_set_up) {
        head_functions_set_up = 1;
        // bm_jpu_set_heap_allocator_functions(bmjpu_heap_alloc_func, bmjpu_heap_free_func, NULL);
    }
    av_log(NULL, AV_LOG_TRACE, "Leave %s\n", __func__);
}

static void bm_jpegdec_buffer_release(void *opaque, uint8_t *data)
{
    bm_opaque_t* bm_opaque = (bm_opaque_t*)opaque;
    BMJpegDecContext* ctx;

    av_log(NULL, AV_LOG_TRACE, "Enter %s\n", __func__);

    if (bm_opaque == NULL)
        return;

    if (bm_opaque->magic_number != MAGIC_JPEG)
        return;

    if (bm_opaque->release_count == 0)
        return;
    bm_opaque->release_count--;
    if (bm_opaque->release_count != 0)
        return;

    ctx = bm_opaque->ctx;
    /* Avoid exception after avcodec_close() */
    if (ctx != NULL && ctx->jpeg_decoder) {
        BmJpuFramebuffer *framebuffer = bm_opaque->framebuffer;
        if (framebuffer != NULL) {
            if (ctx->hw_accel==0) {
#ifndef BM_PCIE_MODE
            #ifdef BOARD_FPGA
                bm_jpu_devm_unmap(bm_opaque->vaddr, framebuffer->dma_buffer->size);
            #else
                /* Unmap the DMA buffer of the decoded picture */
                bm_mem_unmap_device_mem(ctx->handle, bm_opaque->vaddr, framebuffer->dma_buffer->size);
            #endif
#endif
            }
            /* Decoded frame is no longer needed,
            * so inform the decoder that it can reclaim it */
            bm_jpu_jpeg_dec_frame_finished(ctx->jpeg_decoder, framebuffer);
        }

        bm_device_mem_t *dma_buffer_to_user = bm_opaque->dma_buffer_to_user;
        if (dma_buffer_to_user != NULL) {
            if (ctx->hw_accel==0) {
#ifndef BM_PCIE_MODE
            #ifdef BOARD_FPGA
                bm_jpu_devm_unmap(bm_opaque->vaddr, dma_buffer_to_user->size);
            #else
                /* Unmap the DMA buffer of the decoded picture */
                bm_mem_unmap_device_mem(ctx->handle, bm_opaque->vaddr, dma_buffer_to_user->size);
            #endif
#endif
            }
            /* free device memory allocated for user */
            bm_free_device(ctx->handle, *(dma_buffer_to_user));
            av_free(dma_buffer_to_user);
            dma_buffer_to_user = NULL;
        }
    }

    if (ctx->hw_accel==0) {
#ifdef BM_PCIE_MODE
        if (bm_opaque->data != NULL)
            av_free(bm_opaque->data);
#endif
    } else {
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
        if (bm_opaque->hwpic)
            av_free(bm_opaque->hwpic);
#endif
    }

    av_free(bm_opaque);

    av_log(NULL, AV_LOG_TRACE, "Leave %s\n", __func__);
}

static int bm_jpegdec_fill_frame(AVCodecContext *avctx,
                                 BmJpuJPEGDecInfo* info,
                                 AVFrame* frame)
{
    BMJpegDecContext* ctx = (BMJpegDecContext *)(avctx->priv_data);
    BmJpuFramebuffer *framebuffer = info->framebuffer;
    bm_device_mem_t *dma_buffer = framebuffer->dma_buffer;
    bm_opaque_t* bm_opaque;
    uint8_t* phys_addr = (uint8_t*)bm_mem_get_device_addr(*(dma_buffer));
    unsigned int data_size = bm_mem_get_device_size(*(dma_buffer));
    int size_yuv[3] = { info->y_size, info->cbcr_size, info->cbcr_size };
    int i;
    int ret;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    frame->width       = avctx->width;
    frame->height      = avctx->height;
    frame->format      = avctx->pix_fmt;
    frame->colorspace  = avctx->colorspace;
    frame->color_range = avctx->color_range;
    frame->pict_type   = AV_PICTURE_TYPE_I;
    frame->key_frame   = 1;

    // copy frame data to user and release internal framebuffer
    if (ctx->framebuffer_recycle) {
        av_log(avctx, AV_LOG_INFO, "framebuffer is recycled, framebuffer size is %zu\n", ctx->framebuffer_size);

        bm_device_mem_t *dma_buffer_to_user = (bm_device_mem_t *)av_mallocz(sizeof(bm_device_mem_t));
        ret = bm_malloc_device_byte_heap_mask(ctx->handle, dma_buffer_to_user, HEAP_MASK_1_2, ctx->framebuffer_size);
        if (ret != BM_SUCCESS)
        {
            av_log(avctx, AV_LOG_ERROR, "bm_malloc_device_byte_heap_mask failed\n");
            return AVERROR(ENOMEM);
        }

        ret = bm_memcpy_d2d_byte(ctx->handle, *(dma_buffer_to_user), 0, *(dma_buffer), 0, ctx->framebuffer_size);
        if (ret != BM_SUCCESS)
        {
            av_log(avctx, AV_LOG_ERROR, "bm_memcpy_d2d_byte failed\n");
            return AVERROR_UNKNOWN;
        }

        bm_jpu_jpeg_dec_frame_finished(ctx->jpeg_decoder, framebuffer);

        framebuffer = NULL;
        dma_buffer = dma_buffer_to_user;
        phys_addr = (uint8_t*)bm_mem_get_device_addr(*(dma_buffer_to_user));
        data_size = ctx->framebuffer_size;

        av_log(avctx, AV_LOG_INFO, "dma buffer physical address is %#lx\n", phys_addr);
    }

    bm_opaque = (bm_opaque_t*)av_mallocz(sizeof(bm_opaque_t));
    bm_opaque->magic_number         = MAGIC_JPEG;
    bm_opaque->ctx                  = ctx;
    bm_opaque->framebuffer          = framebuffer;
    bm_opaque->dma_buffer_to_user   = ctx->framebuffer_recycle ? dma_buffer : NULL;
    bm_opaque->release_count        = 3;

    /* For bm hw accel framework */
    if (ctx->hw_accel) {
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
        AVBmCodecFrame* hwpic = av_mallocz(sizeof(AVBmCodecFrame));
        if (hwpic == NULL) {
            av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
            return AVERROR(ENOMEM);
        }

        hwpic->type        = 0;
        hwpic->buffer      = dma_buffer;

        hwpic->data[0]     = phys_addr + info-> y_offset;
        hwpic->data[1]     = phys_addr + info->cb_offset;
        hwpic->data[2]     = phys_addr + info->cr_offset;

        hwpic->linesize[0] = info->y_stride;
        hwpic->linesize[1] =
        hwpic->linesize[2] = info->cbcr_stride;

        bm_opaque->hwpic   = hwpic;

        for (i=0; i<3; i++) {
            frame->buf[i] = av_buffer_create(hwpic->data[i], size_yuv[i],
                                             bm_jpegdec_buffer_release, (void*)bm_opaque,
                                             AV_BUFFER_FLAG_READONLY);
            frame->data[i] = hwpic->data[i];
            if (i==0)
                frame->linesize[i] = info->   y_stride;
            else
                frame->linesize[i] = info->cbcr_stride;
        }
        frame->data[4]     = (uint8_t*)hwpic;

        // TODO debug soc mode
        if (avctx->hw_frames_ctx) {
            frame->hw_frames_ctx = av_buffer_ref(avctx->hw_frames_ctx);
            if (!frame->hw_frames_ctx)
                return AVERROR(ENOMEM);
        }
#endif
    } else {
        uint8_t* virt_addr;
        uint8_t* yuv_virt_addr[3];
#ifdef BM_PCIE_MODE
        virt_addr = av_mallocz(data_size);
        if (!virt_addr) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate buffer\n");
            return AVERROR(ENOMEM);
        }
        if (!ctx->zero_copy) {
            // TODO extra cost when process only via HW device
            ret = bm_memcpy_d2s_partial(ctx->handle, virt_addr, *(dma_buffer), data_size);
            if (ret != BM_SUCCESS) {
                av_free(virt_addr);
                av_log(avctx, AV_LOG_ERROR, "Failed to call bm_memcpy_d2s\n");
                return AVERROR_UNKNOWN;
            }
        }
#else
    #ifdef BOARD_FPGA
        virt_addr = (uint8_t*)bm_jpu_devm_map(phys_addr, data_size);
    #else
        /* Map the DMA buffer of the decoded picture */
        // TODO extra cost when process only via HW
        unsigned long long p_vaddr = 0;
        int ret = bm_mem_mmap_device_mem(ctx->handle, dma_buffer, &p_vaddr);
        if (ret != BM_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR, "Failed to call bm_mem_mmap_device_mem\n");
            return AVERROR_UNKNOWN;
        }
        virt_addr = (uint8_t*)p_vaddr;
    #endif

    #if 0
        FILE *fp = fopen("dump.yuv", "wb");
        fwrite(virt_addr, sizeof(uint8_t), data_size, fp);
        fclose(fp);
    #endif
#endif
        yuv_virt_addr[0] = virt_addr + info->y_offset;
        yuv_virt_addr[1] = virt_addr + info->cb_offset;
        yuv_virt_addr[2] = virt_addr + info->cr_offset;

#ifdef BM_PCIE_MODE
        bm_opaque->data = virt_addr;
#else
        bm_opaque->vaddr = virt_addr;
#endif

        for (i=0; i<3; i++) {
            frame->buf[i] = av_buffer_create(yuv_virt_addr[i], size_yuv[i],
                                             bm_jpegdec_buffer_release, (void*)bm_opaque,
                                             AV_BUFFER_FLAG_READONLY);
            frame->data[i] = yuv_virt_addr[i];
            if (i==0)
                frame->linesize[i] = info->y_stride;
            else
                frame->linesize[i] = info->cbcr_stride;
        }

        frame->data[4] = phys_addr + info-> y_offset;
        frame->data[5] = phys_addr + info->cb_offset;
        frame->data[6] = phys_addr + info->cr_offset;
        frame->linesize[4] = info->y_stride;
        frame->linesize[5] = info->cbcr_stride;
        frame->linesize[6] = info->cbcr_stride;
        frame->opaque = (void*)framebuffer;
    }

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;
}

static av_cold int bm_jpegdec_init(AVCodecContext *avctx)
{
    BMJpegDecContext* ctx = (BMJpegDecContext*)(avctx->priv_data);
    BmJpuDecOpenParams open_params;
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
    AVBmCodecDeviceContext *bmcodec_device_hwctx = NULL;
#endif
    int ret = 0;

    // av_log_set_level(AV_LOG_TRACE);

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    av_log(avctx, AV_LOG_DEBUG, "bs_buffer_size: %dK\n",
           ctx->bs_buffer_size);
    av_log(avctx, AV_LOG_DEBUG, "num_extra_framebuffers: %d\n",
           ctx->num_extra_framebuffers);

    ctx->old_pix_fmt  = AV_PIX_FMT_NONE;
    if (avctx->pix_fmt != AV_PIX_FMT_NONE) {
        avctx->sw_pix_fmt = avctx->pix_fmt;
        pix_fmts[1] = avctx->sw_pix_fmt;  // change to current format
    } else {
        avctx->sw_pix_fmt = AV_PIX_FMT_YUVJ420P;
    }

    ret = ff_get_format(avctx, pix_fmts);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed!\n");
        return ret;
    }
    av_log(avctx, AV_LOG_DEBUG, "ff_get_format: %s.\n", av_get_pix_fmt_name(ret));
    avctx->pix_fmt = ret;

    avctx->chroma_sample_location = AVCHROMA_LOC_CENTER;
    avctx->colorspace             = AVCOL_SPC_BT470BG;
    avctx->color_range            = AVCOL_RANGE_JPEG;

    ctx->hw_accel = 0;
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
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

    if (ctx->hw_accel == 0) {
        avctx->sw_pix_fmt =
        avctx->pix_fmt    = AV_PIX_FMT_YUVJ420P;
    }

    bmjpu_setup_logging();

    bmjpu_setup_heap_allocator_functions();

    ret = bm_jpu_dec_load(ctx->soc_idx);
    if (ret != BM_JPU_DEC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to call bm_jpu_dec_load\n");
        return AVERROR_UNKNOWN;
    }

    memset(&open_params, 0, sizeof(BmJpuDecOpenParams));
    // FIXME: set width and height
    open_params.frame_width  = avctx->width;
    open_params.frame_height = avctx->height;
    if (ctx->hw_accel)
        open_params.chroma_interleave = 0;
    else
        open_params.chroma_interleave = ctx->chroma_interleave;
    open_params.bs_buffer_size = ctx->bs_buffer_size*1024;
    open_params.device_index = ctx->soc_idx;
    open_params.color_format = BM_JPU_COLOR_FORMAT_BUTT;

    ctx->jpeg_decoder = NULL;
    ret = bm_jpu_jpeg_dec_open(&(ctx->jpeg_decoder), &open_params,
                               ctx->num_extra_framebuffers);
    if (ret != BM_JPU_DEC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "Failed to call bm_jpu_jpeg_dec_open\n");
        return AVERROR_INVALIDDATA;
    }

    if (ctx->perf) {
        ctx->total_time  = 0.0f;
        ctx->total_frame = 0L;
    }

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;
}

static av_cold int bm_jpegdec_close(AVCodecContext *avctx)
{
    BMJpegDecContext* ctx = (BMJpegDecContext *)(avctx->priv_data);
    int ret;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    if (ctx->perf) {
        if (ctx->total_time > 0.0f) {
            av_log(avctx, AV_LOG_INFO, "Frames decoded: %ld. Decoding speed: %.1ffps. Time used: %.3fsec.\n",
                   ctx->total_frame, ctx->total_frame*1000/ctx->total_time, ctx->total_time/1000.0f);
        }
    }

    if (ctx->jpeg_decoder) {
        bm_jpu_jpeg_dec_close(ctx->jpeg_decoder);
        ctx->jpeg_decoder = NULL; /* mark after avcodec_close() */

        ret = bm_jpu_dec_unload(ctx->soc_idx);
        if (ret != BM_JPU_DEC_RETURN_CODE_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to call bm_jpu_dec_unload\n");
            return AVERROR_UNKNOWN;
        }
    }

    bm_dev_free(ctx->handle);
    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;
}

static int bm_jpegdec_decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                                   AVPacket *avpkt)
{
    BMJpegDecContext* ctx = (BMJpegDecContext *)(avctx->priv_data);
    AVFrame* frame = (AVFrame*)data;
    BmJpuJPEGDecInfo info;
    BmJpuDecReturnCodes dec_ret;
    struct timeval ps = {0};
    int ret = 0;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    // TODO
    if (frame->buf[0] != NULL) {
        av_log(avctx, AV_LOG_ERROR, "can't fill frame buffer, please check it.\n");
        return AVERROR_UNKNOWN;
    }

    if (ctx->perf) {
        gettimeofday(&ps, NULL);
    }

    *got_frame = 0;

#if 0
    FILE *fp = fopen("dump.jpg", "wb");
    fwrite(avpkt->data, sizeof(uint8_t), avpkt->size, fp);
    fclose(fp);
#endif

    dec_ret = bm_jpu_jpeg_dec_decode(ctx->jpeg_decoder, avpkt->data, avpkt->size);
    if (dec_ret != BM_JPU_DEC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR,
               "Failed to call bm_jpu_jpeg_dec_decode: %s\n",
               bm_jpu_dec_error_string(dec_ret));
        return AVERROR_INVALIDDATA;
    }

    /* Get some information about the the frame */
    memset(&info, 0, sizeof(BmJpuJPEGDecInfo));
    bm_jpu_jpeg_dec_get_info(ctx->jpeg_decoder, &info);

    // av_log(avctx, AV_LOG_DEBUG, "aligned frame size: %u x %u\n",
    //        info.aligned_frame_width, info.aligned_frame_height);
    av_log(avctx, AV_LOG_DEBUG, "pixel actual frame size: %u x %u\n",
           info.actual_frame_width, info.actual_frame_height);
    av_log(avctx, AV_LOG_DEBUG, "pixel Y/Cb/Cr stride: %u/%u/%u\n",
           info.y_stride, info.cbcr_stride, info.cbcr_stride);
    av_log(avctx, AV_LOG_DEBUG, "Y/Cb/Cr size: %u/%u/%u\n",
           info.y_size, info.cbcr_size, info.cbcr_size);
    av_log(avctx, AV_LOG_DEBUG, "Y/Cb/Cr offset: %u/%u/%u\n",
           info.y_offset, info.cb_offset, info.cr_offset);
    av_log(avctx, AV_LOG_DEBUG, "color format: %s\n",
           bm_jpu_color_format_string(info.color_format));
    av_log(avctx, AV_LOG_DEBUG, "chroma interleave: %d\n",
           info.chroma_interleave);

    if (info.framebuffer == NULL) {
        av_log(avctx, AV_LOG_ERROR,
               "could not decode this JPEG image : no framebuffer returned\n");
        return AVERROR_INVALIDDATA;
    }

    ret = ff_set_dimensions(avctx, info.actual_frame_width, info.actual_frame_height);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_set_dimensions failed!\n");
        return ret;
    }

    switch (info.color_format) {
    case BM_JPU_COLOR_FORMAT_YUV420:
        if (info.chroma_interleave == 0)
            avctx->sw_pix_fmt = AV_PIX_FMT_YUVJ420P;
        else
            avctx->sw_pix_fmt = AV_PIX_FMT_NV12;
        break;
    case BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL:
        if (info.chroma_interleave == 0)
            avctx->sw_pix_fmt = AV_PIX_FMT_YUVJ422P;
        else
            avctx->sw_pix_fmt = AV_PIX_FMT_NV16;
        break;
    case BM_JPU_COLOR_FORMAT_YUV444:
        avctx->sw_pix_fmt = AV_PIX_FMT_YUVJ444P;
        break;
    case BM_JPU_COLOR_FORMAT_YUV400:
        avctx->sw_pix_fmt = AV_PIX_FMT_GRAY8;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR,
               "Unsupported color format!\n");
        return AVERROR_PATCHWELCOME;
    }

    if (avctx->width  != info.actual_frame_width ||
        avctx->height != info.actual_frame_height ||
        ctx->old_pix_fmt != avctx->sw_pix_fmt) {
        avctx->width  = info.actual_frame_width;
        avctx->height = info.actual_frame_height;
        ctx->old_pix_fmt = avctx->sw_pix_fmt;

        if (ctx->hw_accel) {
            pix_fmts[1] = avctx->sw_pix_fmt;

            ret = ff_get_format(avctx, pix_fmts);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "ff_get_format failed!\n");
                return ret;
            }

            avctx->pix_fmt = ret;
        } else {
            avctx->pix_fmt = avctx->sw_pix_fmt;
        }
    }

    av_log(avctx, AV_LOG_DEBUG, "avctx->sw_pix_fmt = %d\n", avctx->sw_pix_fmt);
    av_log(avctx, AV_LOG_DEBUG, "avctx->pix_fmt = %d\n", avctx->pix_fmt);

    avctx->frame_size = info.actual_frame_width * info.actual_frame_height;

    bm_jpegdec_fill_frame(avctx, &info, frame);
    frame->pts     = avpkt->pts;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
    frame->pkt_pts = avpkt->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    frame->pkt_dts = avpkt->dts;

    *got_frame = 1;

    if (ctx->perf) {
        struct timeval pe;
        gettimeofday(&pe, NULL);
        ctx->total_time += ((pe.tv_sec*1000.0 + pe.tv_usec/1000.0) -
                            (ps.tv_sec*1000.0 + ps.tv_usec/1000.0));
        ctx->total_frame++;
    }

    av_log(avctx, AV_LOG_DEBUG,
           "frame %4d: avpkt size = %8d, pts = %ld, dts = %ld\n",
           avctx->frame_number, avpkt->size, avpkt->pts, avpkt->dts);

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return avpkt->size;
}

static av_cold void bm_jpegdec_flush(AVCodecContext *avctx)
{
    BMJpegDecContext* ctx = (BMJpegDecContext *)(avctx->priv_data);
    BmJpuJPEGDecoder *jpeg_decoder = ctx->jpeg_decoder;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    bm_jpu_jpeg_dec_flush(jpeg_decoder);

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);
}

#define OFFSET(x) offsetof(BMJpegDecContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
#define MAX_FRAMEBUFFER_SIZE (8192 * 8192 * 4)
#define DEFAULT_FRAMEBUFFER_SIZE (1920 * 1080 * 3 / 2)

static const AVOption options[] = {
    { "bs_buffer_size", "the bitstream buffer size (Kbytes) for bm jpeg decoder",
        OFFSET(bs_buffer_size), AV_OPT_TYPE_INT, {.i64 = 5120}, 0, INT_MAX, FLAGS },
    { "chroma_interleave", "chroma interleave of output frame for bm jpeg decoder",
        OFFSET(chroma_interleave), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "cbcr_interleave", "CbCr interleave of output frame for bm jpeg decoder",
        OFFSET(chroma_interleave), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "num_extra_framebuffers", "the number of extra frame buffer for jpeg decoder (0 for still jpeg, at least 2 for motion jpeg)",
        OFFSET(num_extra_framebuffers), AV_OPT_TYPE_INT, {.i64 = 2}, 0, INT_MAX, FLAGS },
    { "sophon_idx", "Sophon device index when running in pcie mode.",
        OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, INT_MIN, INT_MAX, FLAGS },
    { "zero_copy", "don't copy the decoded image back to CPU in pcie mode",
        OFFSET(zero_copy), AV_OPT_TYPE_FLAGS, {.i64 = 1}, 0, 1, FLAGS },
    { "perf", "flag to indicate if do the performance testing",
        OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "framebuffer_recycle", "flag to indicate if recycle the framebuffer in bm jpeg decoder",
        OFFSET(framebuffer_recycle), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "framebuffer_size", "the framebuffer size for bm jpeg decoder (valid if framebuffer_recycle is 1)",
        OFFSET(framebuffer_size), AV_OPT_TYPE_INT, {.i64 = DEFAULT_FRAMEBUFFER_SIZE}, 0, MAX_FRAMEBUFFER_SIZE, FLAGS },
    { NULL },
};

#if defined(BM1684) || defined(BM1686) || defined(BM1688)
static const AVCodecHWConfigInternal *bmcodec_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_BMCODEC,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX | AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX, // TODO
            .device_type = AV_HWDEVICE_TYPE_BMCODEC,
        },
        .hwaccel = &ff_bmcodec_hwaccel,
    },
    NULL
};
#endif

static const AVClass bm_jpegdec_class = {
    .class_name = "jpeg_bm_decoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_jpeg_bm_decoder = {
    .p.name           = "jpeg_bm",
    CODEC_LONG_NAME("BM JPEG DECODER"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_MJPEG,
    .priv_data_size = sizeof(BMJpegDecContext),
    .init           = bm_jpegdec_init,
    .close          = bm_jpegdec_close,
    FF_CODEC_DECODE_CB(bm_jpegdec_decode_frame),
    .flush          = bm_jpegdec_flush,
    .p.capabilities   = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_AVOID_PROBING,
#if defined(BM1684) || defined(BM1686) || defined(BM1688)
    .hw_configs     = bmcodec_hw_configs,
#endif
    .p.priv_class     = &bm_jpegdec_class,
    .p.wrapper_name   = "bm"
};

