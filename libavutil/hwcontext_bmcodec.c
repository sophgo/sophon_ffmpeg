/*
 * Bitmain hardware context
 *
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

/*
 * The code associated with .hw_type
 */

#include <unistd.h>
#include "config.h"

#include "buffer.h"
#include "common.h"
#include "hwcontext.h"
#include "hwcontext_internal.h"
#include "hwcontext_bmcodec.h"
#include "libavcodec/avcodec.h"
#include "mem.h"
#include "pixfmt.h"
#include "pixdesc.h"
#include "imgutils.h"

#include "bm_vpuenc_interface.h"

#define BM_FRAME_ALIGNMENT  8
#define HEAP_MASK_1_2 0x06

static const enum AVPixelFormat supported_sw_formats[] = {
    AV_PIX_FMT_NV21,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_RGB24,
    AV_PIX_FMT_BGR24,
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_RGBP,
    AV_PIX_FMT_BGRP,
};

static int bmcodec_device_create(AVHWDeviceContext *ctx, const char *device,
                                 AVDictionary *opts, int flags)
{
    AVBmCodecDeviceContext *hwctx = ctx->hwctx;
    int ret = 0;
#if defined(BM_PCIE_MODE)
    char device_name[32] = {0};
    int device_idx = 0;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (device)
        device_idx = strtol(device, NULL, 0);
#ifdef __linux__
    sprintf(device_name, "/dev/bm-sophon%d", device_idx);
    ret = access(device_name, F_OK);
    if (ret!=0) {
        av_log(ctx, AV_LOG_ERROR, "Please check device: %s\n", device_name);
        return AVERROR(ENOENT);
    }
#endif
    av_log(ctx, AV_LOG_INFO, "The device: %s\n", device_name);

    /* initialize the device */
    hwctx->device_idx = device_idx;
#else
    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    /* initialize the device */
    hwctx->device_idx = 0;
#endif

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return ret;
}

static int bmcodec_device_init(AVHWDeviceContext *ctx)
{
    AVBmCodecDeviceContext *hwctx = ctx->hwctx;
    int ret = 0;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    ret = bm_dev_request(&hwctx->handle, hwctx->device_idx);
    if(ret != BM_SUCCESS)
    {
        av_log(ctx, AV_LOG_ERROR, "bmlib create handle failed\n");
        ret = -1;
        goto Exit;
    }

Exit:
    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
    return ret;
}

static void bmcodec_device_uninit(AVHWDeviceContext *ctx)
{
    AVBmCodecDeviceContext *hwctx = ctx->hwctx;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if(hwctx->handle)
        bm_dev_free(hwctx->handle);

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
}


static int bmcodec_frames_get_constraints(AVHWDeviceContext* ctx,
                                          const void* hwconfig,
                                          AVHWFramesConstraints* constraints)
{
    int i;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    constraints->valid_sw_formats = av_malloc_array(FF_ARRAY_ELEMS(supported_sw_formats) + 1,
                                                    sizeof(*constraints->valid_sw_formats));
    if (!constraints->valid_sw_formats) {
        av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < FF_ARRAY_ELEMS(supported_sw_formats); i++)
        constraints->valid_sw_formats[i] = supported_sw_formats[i];
    constraints->valid_sw_formats[FF_ARRAY_ELEMS(supported_sw_formats)] = AV_PIX_FMT_NONE;

    constraints->valid_hw_formats = av_malloc_array(2, sizeof(*constraints->valid_hw_formats));
    if (!constraints->valid_hw_formats) {
        av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
        return AVERROR(ENOMEM);
    }

    constraints->valid_hw_formats[0] = AV_PIX_FMT_BMCODEC;
    constraints->valid_hw_formats[1] = AV_PIX_FMT_NONE;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;
}

static void bmcodec_buffer_free(void* opaque, uint8_t* data)
{
    AVHWFramesContext* ctx = opaque;
    AVBmCodecDeviceContext* hwctx = ctx->device_ctx->hwctx;
    AVBmCodecFrame   *hwpic  = (AVBmCodecFrame*)data;
    int ret;
    bm_device_mem_t *buffer;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    /* call bm free function to free buffer */
    buffer = (bm_device_mem_t*)hwpic->buffer;
    if(buffer){
        if(buffer->u.device.dmabuf_fd == -1)
        {
            av_free(buffer);
        }else{
            bm_free_device(hwctx->handle, *buffer);
            av_free(buffer);
        }
    }

    av_free(hwpic);

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
}

static AVBufferRef* bmcodec_pool_alloc(void *opaque, int size)
{
    AVHWFramesContext     *ctx = opaque;
    AVBmCodecDeviceContext *hwctx = ctx->device_ctx->hwctx;
    AVBmCodecFramesContext *frmctx = ctx->hwctx;

    AVBufferRef *ref = NULL;
    AVBmCodecFrame   *hwpic  = NULL;
    int ret;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    /* call bm allocation function to allocate buffer */
    bm_device_mem_t *buffer = (bm_device_mem_t *)av_mallocz(sizeof(bm_device_mem_t));
    ret = bmvpu_malloc_device_byte_heap(hwctx->handle, buffer, size, HEAP_MASK_1_2, 1);
    if(ret != BM_SUCCESS)
    {
        av_free(buffer);
        av_log(ctx, AV_LOG_ERROR, "[%s,%d] bmvpu_malloc_device_byte_heap failed.\n", __func__, __LINE__);
        goto fail;
    }

    frmctx->type = 1;

    hwpic = av_mallocz(sizeof(AVBmCodecFrame));
    if (hwpic == NULL) {
        av_log(ctx, AV_LOG_ERROR, "[%s,%d] av_mallocz failed.\n", __func__, __LINE__);
        goto fail;
    }
    hwpic->type   = 1;
    hwpic->maptype= 0;
    hwpic->handle = hwctx->handle;
    hwpic->buffer = buffer;
    hwpic->padded = 0;

    ref = av_buffer_create((uint8_t*)hwpic, size, bmcodec_buffer_free, ctx, 0);
    if (!ref) {
        av_log(ctx, AV_LOG_ERROR, "[%s,%d] av_buffer_create failed.\n", __func__, __LINE__);
        /* call bm free function to free buffer */
        bm_free_device(hwctx->handle, *((bm_device_mem_t *)buffer));
        av_free(hwpic);
        goto fail;
    }

fail:
    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
    return ref;
}

static int bmcodec_image_fill_arrays(uint8_t *dst_data[4], int dst_linesize[4],
                                     const uint8_t *src, enum AVPixelFormat pix_fmt,
                                     int width, int height, int codec_id)
{
    int ret, i;
    ret = av_image_check_size(width, height, 0, NULL);
    if (ret < 0)
        return ret;
    ret = av_image_fill_linesizes(dst_linesize, pix_fmt, width);
    if (ret < 0)
        return ret;
    if (codec_id != AV_CODEC_ID_MJPEG)
        for (i = 1; i < 4; i++)
            dst_linesize[i] = FFALIGN(dst_linesize[i], BM_FRAME_ALIGNMENT);

    if (pix_fmt == AV_PIX_FMT_YUV420P)
        dst_linesize[0] = dst_linesize[1]*2;
    else if (pix_fmt == AV_PIX_FMT_NV12)
        dst_linesize[0] = dst_linesize[1];
    else
        dst_linesize[0] = FFALIGN(dst_linesize[0], BM_FRAME_ALIGNMENT);

    ret = av_image_fill_pointers(dst_data, pix_fmt, height, (uint8_t *)src, dst_linesize);
    if (pix_fmt == AV_PIX_FMT_GRAY8 && dst_data[1]) // to defense FF_API_PSEDUOPAL setting
        dst_data[1] = NULL;

    return ret;
}

static int bmcodec_image_get_buffer_size(enum AVPixelFormat pix_fmt, int width, int height, int codec_id)
{
    uint8_t *data[4];
    int linesize[4];
    int ret;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    if (!desc)
        return AVERROR(EINVAL);

    ret = av_image_check_size(width, height, 0, NULL);
    if (ret < 0)
        return ret;

    return bmcodec_image_fill_arrays(data, linesize, NULL, pix_fmt, width, height, codec_id);
}

static int bmcodec_frames_init(AVHWFramesContext *ctx)
{
    BmCodecFramesContext *priv = ctx->internal->priv;
    int i;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);
    for (i = 0; i < FF_ARRAY_ELEMS(supported_sw_formats); i++) {
        if (ctx->sw_format == supported_sw_formats[i])
            break;
    }
    if (i == FF_ARRAY_ELEMS(supported_sw_formats)) {
        av_log(ctx, AV_LOG_ERROR, "Pixel format '%s' is not supported\n",
               av_get_pix_fmt_name(ctx->sw_format));
        return AVERROR(ENOSYS);
    }

    av_pix_fmt_get_chroma_sub_sample(ctx->sw_format, &priv->shift_width, &priv->shift_height);

    if (!ctx->pool) {
        int size = bmcodec_image_get_buffer_size(ctx->sw_format, ctx->width, ctx->height, priv->codec_id);
        if (size < 0)
            return size;

        av_log(ctx, AV_LOG_DEBUG, "[%s,%d] sw format=%s, width=%d, height=%d, alignment=%d\n",
               __func__, __LINE__, av_get_pix_fmt_name(ctx->sw_format), ctx->width, ctx->height, BM_FRAME_ALIGNMENT);

        ctx->internal->pool_internal = av_buffer_pool_init2(size, ctx, bmcodec_pool_alloc, NULL);
        if (!ctx->internal->pool_internal) {
            av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
            return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static int bmcodec_get_buffer(AVHWFramesContext *ctx, AVFrame *frame)
{
    AVBmCodecFrame   *hwpic = NULL;
    bm_device_mem_t *buffer = NULL;
    BmCodecFramesContext *priv = ctx->internal->priv;
    int i, ret;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    frame->buf[0] = av_buffer_pool_get(ctx->pool);
    if (!frame->buf[0]) {
        av_log(ctx, AV_LOG_ERROR, "[%s,%d] av_buffer_pool_get failed!\n", __func__, __LINE__);
        return AVERROR(ENOMEM);
    }

    /* A whole buffer is allocated */
    frame->data[4] = frame->buf[0]->data;

    hwpic = (AVBmCodecFrame*)(frame->buf[0]->data);
    buffer = (bm_device_mem_t*)hwpic->buffer;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] paddr=0x%lx, size=%d\n",
           __func__, __LINE__, buffer->u.device.device_addr, buffer->size);

    ret = bmcodec_image_fill_arrays(hwpic->data, hwpic->linesize,
                                    (uint8_t*)(buffer->u.device.device_addr), ctx->sw_format,
                                    ctx->width, ctx->height, priv->codec_id);
    if (ret < 0)
        return ret;

    for (i=0; i<3; i++) {
        frame->data[i]     = hwpic->data[i];
        frame->linesize[i] = hwpic->linesize[i];
    }

    frame->format  = AV_PIX_FMT_BMCODEC;
    frame->width   = ctx->width;
    frame->height  = ctx->height;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;
}

/*-----------------------------------------------------*
 *
 * the transfering functions will be supported later.
 *
 *-----------------------------------------------------*/
static int bm_transfer_get_formats(AVHWFramesContext *ctx,
                                   enum AVHWFrameTransferDirection dir,
                                   enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts = av_malloc_array(2, sizeof(*fmts));
    if (!fmts)
        return AVERROR(ENOMEM);

    fmts[0] = ctx->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    *formats = fmts;
    return 0;
}

#if !defined(BM_PCIE_MODE)
static void bm_unmap_frame(AVHWFramesContext *ctx, HWMapDescriptor *hwmap)
{
#if 0
    AVBmCodecFrame* hwpic = (AVBmCodecFrame*)hwmap->source->data[4];
    bm_ion_buffer_t*    p = (bm_ion_buffer_t*)hwpic->buffer;
#endif
    AVBmCodecDeviceContext *hwctx = ctx->device_ctx->hwctx;
    bm_device_mem_t* p = hwmap->priv;

    if (p==NULL || p->u.device.dmabuf_fd ==-1) // reserved memory need not munmap
        return;

    /* unmap virtual buffer */

    /* if write flags, flush pa */
    //if (p->map_flags | BM_ION_MAPPING_FLAG_WRITE)
    // bm_mem_flush_device_mem(hwctx->handle, p);

    bm_mem_unmap_device_mem(hwctx->handle, hwctx->map_vaddr, p->size);
}

static int bm_map_frame(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src,
                        int flags)
{
    AVBmCodecFrame*   hwpic = (AVBmCodecFrame*)src->data[4];
    bm_device_mem_t* buffer = (bm_device_mem_t*)hwpic->buffer;
    AVBmCodecDeviceContext *hwctx = ctx->device_ctx->hwctx;
    BmCodecFramesContext *priv = ctx->internal->priv;
    uint32_t map_flags = 0;
    int ret;

    if (buffer==NULL) {
        av_log(ctx, AV_LOG_ERROR,
               "The hw buffer is from private memory, unsupported now\n");
        return AVERROR_EXTERNAL;
    }

    av_log(ctx, AV_LOG_DEBUG, "<%s,%d> type: %d, buffer: %p\n",
           __func__, __LINE__, hwpic->type, buffer);

    if (ctx->sw_format != dst->format) {
        av_log(ctx, AV_LOG_ERROR,
               "The input pixel format(%s) does NOT match the output pixel format(%s).\n",
               av_get_pix_fmt_name(ctx->sw_format), av_get_pix_fmt_name(dst->format));
        return AVERROR_INVALIDDATA;
    }
    if (flags & AV_HWFRAME_MAP_DIRECT) { // TODO
        av_log(ctx, AV_LOG_ERROR,
               "For now, Don't support direct map mode.\n");
        return AVERROR_INVALIDDATA;
    }

    hwctx->map_vaddr = hwpic->buffer_vaddr;
    if (buffer->u.device.dmabuf_fd != -1){ // ion memory
         unsigned long long vmem = 0;
        bm_mem_mmap_device_mem_no_cache(hwctx->handle, buffer, &vmem);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "bm_ion_map_buffer failed\n");
            return AVERROR_EXTERNAL;
        }
        hwctx->map_vaddr = (uint8_t*)vmem;

        // bm_mem_invalidate_device_mem(hwctx->handle, buffer);
        // if (ret < 0) {
        //     av_log(ctx, AV_LOG_ERROR, "bm_ion_invalidate_buffer failed\n");
        //     return AVERROR_EXTERNAL;
        // }
    }

    ret = bmcodec_image_fill_arrays(dst->data, dst->linesize,
                                    hwctx->map_vaddr, ctx->sw_format,
                                    ctx->width, ctx->height,
                                    priv->codec_id);
    if (ret < 0)
        goto fail;

    ret = ff_hwframe_map_create(src->hw_frames_ctx, dst, src,
                                bm_unmap_frame,
                                (void *)buffer);
    if (ret < 0)
        goto fail;

    dst->width  = src->width;
    dst->height = src->height;

    return 0;

fail:
    if (buffer->u.device.dmabuf_fd !=-1)
        bm_mem_unmap_device_mem(hwctx->handle, hwctx->map_vaddr, buffer->size);

    return ret;
}
#endif

#ifdef BM_PCIE_MODE
static int bm_image_download(AVHWFramesContext *ctx, int soc_idx,
                           uint8_t *dst_data[4], int dst_linesizes[4],
                           const uint64_t src_data[4], int src_linesizes[4],
                           enum AVPixelFormat pix_fmt, int width, int height)
{
    AVBmCodecDeviceContext *hwctx = ctx->device_ctx->hwctx;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    bm_device_mem_t src_mem;
    int i, planes_nb = 0;
    unsigned int size = 0;
    int ret;

    for (i = 0; i < desc->nb_components; i++)
        planes_nb = FFMAX(planes_nb, desc->comp[i].plane + 1);

    for (i = 0; i < planes_nb; i++) {
        uint8_t*  dst          = dst_data[i];
        ptrdiff_t dst_linesize = dst_linesizes[i];
        uint64_t  src          = (uint64_t)src_data[i];
        ptrdiff_t src_linesize = src_linesizes[i];

        int h = height;
        ptrdiff_t bwidth = av_image_get_linesize(pix_fmt, width, i);
        if (bwidth < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_image_get_linesize failed\n");
            return -1;
        }
        if (i == 1 || i == 2) {
            h = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);
        }
        if (dst_linesize != src_linesize) {
            av_log(NULL, AV_LOG_ERROR, "dst_linesize (%zd) is not equal to src_linesize (%zd)\n",
                   dst_linesize, src_linesize);
            return -1;
        }

        av_log(NULL, AV_LOG_DEBUG,
               "plane %d: bwidth %zd, dst_linesize %zd, src_linesize %zd\n",
               i, bwidth, dst_linesize, src_linesize);


        size = src_linesize * h;
        src_mem = bm_mem_from_device(src, size);
        bm_memcpy_d2s_partial(hwctx->handle, dst,src_mem, size);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "bmvpu_upload_data failed\n");
            return -1;
        }
    }

    return 0;
}

static int bm_image_upload(AVHWFramesContext *ctx,int soc_idx,
                           uint64_t dst_data[4], int dst_linesizes[4],
                           uint8_t *src_data[4], int src_linesizes[4],
                           enum AVPixelFormat pix_fmt, int width, int height)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    AVBmCodecDeviceContext *hwctx  = ctx->device_ctx->hwctx;
    int i, planes_nb = 0;
    unsigned int size;
    bm_device_mem_t dst_mem;
    int ret;

    for (i = 0; i < desc->nb_components; i++)
        planes_nb = FFMAX(planes_nb, desc->comp[i].plane + 1);

    for (i = 0; i < planes_nb; i++) {
        uint8_t*  src          = src_data[i];
        ptrdiff_t src_linesize = src_linesizes[i];
        uint64_t  dst          = (uint64_t)dst_data[i];
        ptrdiff_t dst_linesize = dst_linesizes[i];

        int h = height;
        ptrdiff_t bwidth = av_image_get_linesize(pix_fmt, width, i);
        if (bwidth < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_image_get_linesize failed\n");
            return -1;
        }
        if (i == 1 || i == 2) {
            h = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);
        }
        if (dst_linesize != src_linesize) {
            av_log(NULL, AV_LOG_ERROR, "dst_linesize (%zd) is not equal to src_linesize (%zd)\n",
                   dst_linesize, src_linesize);
            return -1;
        }

        av_log(NULL, AV_LOG_DEBUG,
               "plane %d: bwidth %zd, dst_linesize %zd, src_linesize %zd\n",
               i, bwidth, dst_linesize, src_linesize);

        size = src_linesize * h;
        dst_mem = bm_mem_from_device(dst, size);
        bm_memcpy_s2d_partial(hwctx->handle, dst_mem , src, size);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "bmvpu_upload_data failed\n");
            return -1;
        }
    }

    return 0;
}
#endif

/* Download hardware frames to system memory. */
static int bm_transfer_data_from(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src)
{
    int ret = 0;
#if defined(BM_PCIE_MODE)
    AVBmCodecFramesContext* hwfrmctx = ctx->hwctx;
    AVBmCodecDeviceContext *bmcodec_device_hwctx = ctx->device_ctx->hwctx;
    int soc_idx = bmcodec_device_hwctx->device_idx;
    AVBmCodecFrame* hwsrc = (AVBmCodecFrame*)src->data[4];
    uint64_t src_data[4] = { (uint64_t)hwsrc->data[0], (uint64_t)hwsrc->data[1], (uint64_t)hwsrc->data[2], 0L };
    int src_linesizes[4] = { hwsrc->linesize[0], hwsrc->linesize[1], hwsrc->linesize[2], 0 };
    uint8_t* dst_data[4] = { dst->data[0], dst->data[1], dst->data[2], NULL };
    int dst_linesizes[4] = { dst->linesize[0], dst->linesize[1], dst->linesize[2], 0 };
    AVFrame *tmp = NULL;

    av_log(ctx, AV_LOG_DEBUG, "<%s,%d> hwfrmctx->type: %d\n",
           __func__, __LINE__, hwfrmctx->type);

    if (dst->width > src->width || dst->height > src->height) {
        av_log(ctx, AV_LOG_DEBUG, "dst: width=%d, height=%d\n", dst->width, dst->height);
        av_log(ctx, AV_LOG_DEBUG, "src: width=%d, height=%d\n", src->width, src->height);
    }

    if (dst->linesize[0] != hwsrc->linesize[0]) {
        av_log(ctx, AV_LOG_DEBUG, "dst->linesize[0] (%d) is not equal to hwsrc->linesize[0] (%d)\n",
               dst->linesize[0], hwsrc->linesize[0]);
    }
    if (dst->linesize[1] != hwsrc->linesize[1]) {
        av_log(ctx, AV_LOG_DEBUG, "dst->linesize[1] (%d) is not equal to hwsrc->linesize[1] (%d)\n",
               dst->linesize[1], hwsrc->linesize[1]);
    }
    if (dst->linesize[2] != hwsrc->linesize[2]) {
        av_log(ctx, AV_LOG_DEBUG, "dst->linesize[2] (%d) is not equal to hwsrc->linesize[2] (%d)\n",
               dst->linesize[2], hwsrc->linesize[2]);
    }

    if (dst->linesize[0] != hwsrc->linesize[0] ||
        dst->linesize[1] != hwsrc->linesize[1] ||
        dst->linesize[2] != hwsrc->linesize[2]) {
        tmp = av_frame_alloc();
        if (!tmp) {
            av_log(ctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
            return AVERROR(ENOMEM);
        }

        tmp->format = dst->format;
        tmp->width  = dst->width; // TODO
        tmp->height = dst->height; // TODO
        tmp->linesize[0] = hwsrc->linesize[0];
        tmp->linesize[1] = hwsrc->linesize[1];
        tmp->linesize[2] = hwsrc->linesize[2];

        ret = av_frame_get_buffer(tmp, 0);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "av_frame_get_buffer failed\n");
            goto fail;
        }

        dst_data[0]      = tmp->data[0];
        dst_data[1]      = tmp->data[1];
        dst_data[2]      = tmp->data[2];
        dst_data[3]      = NULL;

        dst_linesizes[0] = tmp->linesize[0];
        dst_linesizes[1] = tmp->linesize[1];
        dst_linesizes[2] = tmp->linesize[2];
        dst_linesizes[3] = 0;
    }

    av_log(ctx, AV_LOG_DEBUG, "bm_image_download begin:\n");
    av_log(ctx, AV_LOG_DEBUG, "buffer = %p\n", src->buf[0]->data);

    av_log(ctx, AV_LOG_TRACE, "src data 0: %p\n", hwsrc->data[0]);
    av_log(ctx, AV_LOG_TRACE, "src data 1: %p\n", hwsrc->data[1]);
    av_log(ctx, AV_LOG_TRACE, "src data 2: %p\n", hwsrc->data[2]);

    av_log(ctx, AV_LOG_TRACE, "src line size 0: %d\n", hwsrc->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "src line size 1: %d\n", hwsrc->linesize[1]);
    av_log(ctx, AV_LOG_TRACE, "src line size 2: %d\n", hwsrc->linesize[2]);

    av_log(ctx, AV_LOG_TRACE, "src width : %d\n", src->width);
    av_log(ctx, AV_LOG_TRACE, "src height: %d\n", src->height);

    av_log(ctx, AV_LOG_TRACE, "%s: downloading ...\n", __func__);

    ret = bm_image_download(ctx, soc_idx,
                            dst_data, dst_linesizes,
                            src_data, src_linesizes,
                            dst->format, src->width, src->height);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "bm_image_download failed\n");
        return AVERROR_EXTERNAL;
    }
    av_log(ctx, AV_LOG_DEBUG, "bm_image_download end\n");

    if (tmp)
        av_frame_copy(dst, tmp);

fail:
    av_frame_free(&tmp);

    return 0;
#else
    AVFrame *map;

    if (dst->width > ctx->width || dst->height > ctx->height)
        return AVERROR(EINVAL);

    map = av_frame_alloc();
    if (!map)
        return AVERROR(ENOMEM);

    map->format = dst->format;

    ret = bm_map_frame(ctx, map, src, AV_HWFRAME_MAP_READ);
    if (ret)
        goto fail;

    map->width  = dst->width;
    map->height = dst->height;

    ret = av_frame_copy(dst, map);
    if (ret)
        goto fail;

    ret = 0;
fail:
    av_frame_free(&map);
    return ret;
#endif
}

/* Upload system memory frames to hardware frames. */
static int bm_transfer_data_to(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src)
{
    int ret = 0;
#if defined(BM_PCIE_MODE)
    AVBmCodecFramesContext* hwfrmctx = ctx->hwctx;
    AVBmCodecDeviceContext* dev_ctx  = ctx->device_ctx->hwctx;
    int soc_idx = dev_ctx->device_idx;
    AVBmCodecFrame* hwdst = (AVBmCodecFrame*)dst->data[4];
    uint8_t* src_data[4] = { src->data[0], src->data[1], src->data[2], NULL };
    int src_linesizes[4] = { src->linesize[0], src->linesize[1], src->linesize[2], 0 };
    uint64_t dst_data[4] = { (uint64_t)hwdst->data[0], (uint64_t)hwdst->data[1], (uint64_t)hwdst->data[2], 0L };
    int dst_linesizes[4] = { hwdst->linesize[0], hwdst->linesize[1], hwdst->linesize[2], 0 };
    AVFrame *tmp = NULL;

    av_log(ctx, AV_LOG_DEBUG, "<%s,%d> hwfrmctx->type: %d\n",
           __func__, __LINE__, hwfrmctx->type);

    if (hwdst->linesize[0] != src->linesize[0]) {
        av_log(ctx, AV_LOG_DEBUG, "hwdst->linesize[0] (%d) is not equal to src->linesize[0] (%d)\n",
               hwdst->linesize[0], src->linesize[0]);
    }
    if (hwdst->linesize[1] != src->linesize[1]) {
        av_log(ctx, AV_LOG_DEBUG, "hwdst->linesize[1] (%d) is not equal to src->linesize[1] (%d)\n",
               hwdst->linesize[1], src->linesize[1]);
    }
    if (hwdst->linesize[2] != src->linesize[2]) {
        av_log(ctx, AV_LOG_DEBUG, "hwdst->linesize[2] (%d) is not equal to src->linesize[2] (%d)\n",
               hwdst->linesize[2], src->linesize[2]);
    }

    if (hwdst->linesize[0] != src->linesize[0] ||
        hwdst->linesize[1] != src->linesize[1] ||
        hwdst->linesize[2] != src->linesize[2]) {
        tmp = av_frame_alloc();
        if (!tmp) {
            av_log(ctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
            return AVERROR(ENOMEM);
        }

        tmp->format = src->format;
        tmp->width  = src->width; // TODO
        tmp->height = src->height; // TODO
        tmp->linesize[0] = hwdst->linesize[0];
        tmp->linesize[1] = hwdst->linesize[1];
        tmp->linesize[2] = hwdst->linesize[2];

        ret = av_frame_get_buffer(tmp, 0);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "av_frame_get_buffer failed\n");
            goto fail;
        }

        av_frame_copy(tmp, src);

        src_data[0] = tmp->data[0];
        src_data[1] = tmp->data[1];
        src_data[2] = tmp->data[2];
        src_data[3] = NULL;

        src_linesizes[0] = tmp->linesize[0];
        src_linesizes[1] = tmp->linesize[1];
        src_linesizes[2] = tmp->linesize[2];
        src_linesizes[3] = 0;
    }

    av_log(ctx, AV_LOG_DEBUG, "bm_image_upload begin:\n");
    av_log(ctx, AV_LOG_DEBUG, "buffer = %p\n", dst->buf[0]->data);

    av_log(ctx, AV_LOG_TRACE, "dst data 0: %p\n", hwdst->data[0]);
    av_log(ctx, AV_LOG_TRACE, "dst data 1: %p\n", hwdst->data[1]);
    av_log(ctx, AV_LOG_TRACE, "dst data 2: %p\n", hwdst->data[2]);

    av_log(ctx, AV_LOG_TRACE, "dst line size 0: %d\n", hwdst->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "dst line size 1: %d\n", hwdst->linesize[1]);
    av_log(ctx, AV_LOG_TRACE, "dst line size 2: %d\n", hwdst->linesize[2]);

    av_log(ctx, AV_LOG_TRACE, "dst width : %d\n", dst->width);
    av_log(ctx, AV_LOG_TRACE, "dst height: %d\n", dst->height);

    /* Copy YUV420P to YUV420P */

    av_log(ctx, AV_LOG_TRACE, "%s: uploading ...\n", __func__);

    ret = bm_image_upload(ctx, soc_idx,
                          dst_data, dst_linesizes,
                          src_data, src_linesizes,
                          src->format, src->width, src->height);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "bm_image_upload failed\n");
        return AVERROR_EXTERNAL;
    }

    av_log(ctx, AV_LOG_TRACE, "%s: uploading ... Done\n", __func__);
fail:
    av_frame_free(&tmp);

    return 0;
#else

    AVFrame *map;

    if (src->width > ctx->width || src->height > ctx->height)
        return AVERROR(EINVAL);

    map = av_frame_alloc();
    if (!map)
        return AVERROR(ENOMEM);
    map->format = src->format;

    ret = bm_map_frame(ctx, map, dst, AV_HWFRAME_MAP_WRITE | AV_HWFRAME_MAP_OVERWRITE);
    if (ret)
        goto fail;

    map->width  = src->width;
    map->height = src->height;

    ret = av_frame_copy(map, src);
    if (ret)
        goto fail;

    ret = 0;
fail:
    av_frame_free(&map);
    return ret;
#endif
}

#ifndef BM_PCIE_MODE
static int bm_map_from(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src, int flags)
{
    int ret = 0;

    if (src->format != AV_PIX_FMT_BMCODEC ||
        ctx->sw_format != dst->format)
        return AVERROR(ENOSYS);

    ret = bm_map_frame(ctx, dst, src, flags);

    return ret;
}

static int bm_map_to(AVHWFramesContext *ctx, AVFrame *dst, const AVFrame *src, int flags)
{
    int ret = 0;
    // TODO
    return ret;
}
#endif

const HWContextType ff_hwcontext_type_bmcodec = {
    .type                   = AV_HWDEVICE_TYPE_BMCODEC,
    .name                   = "bmcodec",

    .device_hwctx_size      = sizeof(AVBmCodecDeviceContext),
    .frames_priv_size       = sizeof(BmCodecFramesContext),
    .frames_hwctx_size      = sizeof(AVBmCodecFramesContext),

    .device_create          = bmcodec_device_create,

    .device_init            = bmcodec_device_init,
    .device_uninit          = bmcodec_device_uninit,

    .frames_get_constraints = bmcodec_frames_get_constraints,
    .frames_init            = bmcodec_frames_init,
    .frames_get_buffer      = bmcodec_get_buffer,

    .transfer_data_from     = bm_transfer_data_from,
    .transfer_data_to       = bm_transfer_data_to,
    .transfer_get_formats   = bm_transfer_get_formats,

#ifndef BM_PCIE_MODE
    .map_from               = bm_map_from,
    .map_to                 = bm_map_to,
#endif

    .pix_fmts = (const enum AVPixelFormat[]){
        AV_PIX_FMT_BMCODEC,
        AV_PIX_FMT_NONE
    },
};
