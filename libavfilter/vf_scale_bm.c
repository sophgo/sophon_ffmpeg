/*
 * Bitmain hardware scaling and format conversion filter
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

/**
 * @file
 * bmcodec scale video filter
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/time.h"
#include "libavutil/bm_list.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "scale_eval.h"
#include "video.h"
#include "bmcv_api_ext_c.h"
#include "bmlib_runtime.h"
#include "bm_vpuenc_interface.h"

#define HEAP_MASK_1_2 0x06

static int bmscale_avimage_fill_arrays(uint8_t *dst_data[4], int dst_linesize[4], const uint8_t *src, enum AVPixelFormat pix_fmt, int width, int height, int align);
static int bmscale_avimage_get_buffer_size(enum AVPixelFormat pix_fmt, int width, int height, int align);

struct ColorSapce2CSCMap {
    enum AVColorSpace color_space;
    enum AVColorRange color_range;
    csc_type_t csc_type_yuv2rgb;
    csc_type_t csc_type_rgb2yuv;
};

static struct ColorSapce2CSCMap g_csc_map[] = {
        {AVCOL_SPC_BT709,     AVCOL_RANGE_MPEG, CSC_YCbCr2RGB_BT709, CSC_RGB2YCbCr_BT709},
        {AVCOL_SPC_BT709,     AVCOL_RANGE_JPEG, CSC_YPbPr2RGB_BT709, CSC_RGB2YPbPr_BT709},
        {AVCOL_SPC_BT470BG,   AVCOL_RANGE_MPEG, CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601},
        {AVCOL_SPC_BT470BG,   AVCOL_RANGE_JPEG, CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601},
        {AVCOL_SPC_SMPTE170M, AVCOL_RANGE_MPEG, CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601},
        {AVCOL_SPC_SMPTE170M, AVCOL_RANGE_JPEG, CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601},
        {AVCOL_SPC_SMPTE240M, AVCOL_RANGE_MPEG, CSC_YCbCr2RGB_BT601, CSC_RGB2YCbCr_BT601},
        {AVCOL_SPC_SMPTE240M, AVCOL_RANGE_JPEG, CSC_YPbPr2RGB_BT601, CSC_RGB2YPbPr_BT601},
};

static int bmvpu_malloc_device_byte_heap(AVHWDeviceContext *ctx, bm_handle_t bm_handle,
                      bm_device_mem_t *pmem, unsigned int size,
                      int heap_id_mask, int high_bit_first)
{
    int ret = 0;
    int i = 0;
    int heap_num = 0;
    ret = bm_get_gmem_total_heap_num(bm_handle, &heap_num);
    if (ret != 0)
    {
        av_log(ctx, AV_LOG_ERROR, "bmvpu_malloc_device_byte_heap failed!\n");
        return -1;
    }

    int available_heap_mask = 0;
    for (i=0; i<heap_num; i++){
        available_heap_mask = available_heap_mask | (0x1 << i);
    }

    int enable_heap_mask = available_heap_mask & heap_id_mask;
    if (enable_heap_mask == 0x0)
    {
        av_log(ctx, AV_LOG_ERROR, "bmvpu_malloc_device_byte_heap failed(heap_mask is 0x0)!\n");
        return -1;
    }
    if (high_bit_first)
    {
        for (i=(heap_num-1); i>=0; i--)
        {
            if ((enable_heap_mask & (0x1<<i)))
            {
                ret = bm_malloc_device_byte_heap(bm_handle, pmem, i, size);
                if (ret != 0)
                {
                    av_log(ctx, AV_LOG_ERROR, "bm_malloc_device_byte_heap failed! size(%d)\n", size);
                }
                return ret;
            }
        }
    }
    else
    {
        for (i=0; i<heap_num; i++)
        {
            if ((enable_heap_mask & (0x1<<i)))
            {
                ret = bm_malloc_device_byte_heap(bm_handle, pmem, i, size);
                if (ret != 0)
                {
                    av_log(ctx, AV_LOG_ERROR, "bm_malloc_device_byte_heap failed! size(%d)\n", size);
                }
                return ret;
            }
        }
    }

    return BM_VPU_ENC_RETURN_CODE_OK;
}

static int bmcv_get_csc_type_by_colorinfo(int color_space, int color_range, int infmt, int outfmt, csc_type_t *p_csc_type)
{
    int ret = 0;
    int inRGB = 0;
    if (infmt == FORMAT_BGR_PACKED ||
        infmt == FORMAT_BGRP_SEPARATE ||
        infmt == FORMAT_RGB_PACKED ||
        infmt == FORMAT_ARGB_PACKED ||
        infmt == FORMAT_ABGR_PACKED ||
        infmt == FORMAT_RGBP_SEPARATE) {
        inRGB = 1;
    }

    int outRGB = 0;
    if (outfmt == FORMAT_BGR_PACKED ||
        outfmt == FORMAT_BGRP_SEPARATE ||
        outfmt == FORMAT_RGB_PACKED ||
        outfmt == FORMAT_ARGB_PACKED ||
        outfmt == FORMAT_ABGR_PACKED ||
        outfmt == FORMAT_RGBP_SEPARATE) {
        outRGB = 1;
    }

    if(inRGB == outRGB){
        *p_csc_type = CSC_MAX_ENUM;
    }
    else{
        ret = -1;
        for(int i = 0; i< (sizeof(g_csc_map)/sizeof(g_csc_map[0])); ++i) {
            if (g_csc_map[i].color_range == color_range && g_csc_map[i].color_space == color_space) {
                ret = 0;
                *p_csc_type = (inRGB == 0 ? g_csc_map[i].csc_type_yuv2rgb:g_csc_map[i].csc_type_rgb2yuv);
                break;
            }
        }
    }
    return ret;
}

static int bmvpp_scale_bmcv(bm_handle_t handle, bm_image src,
                            bmcv_rect_t* loca, bm_image* dst,
                            int left, int top, int right, int bottom,
                            bmcv_resize_algorithm resize_method,
                            csc_type_t csc_type){
    int ret = 0;
    bmcv_padding_attr_t padding_attr;
    padding_attr.dst_crop_stx = left;
    padding_attr.dst_crop_sty = top;
    padding_attr.dst_crop_w = dst->width - left - right;
    padding_attr.dst_crop_h = dst->height - top - bottom;
    padding_attr.padding_b = 0;
    padding_attr.padding_g = 0;
    padding_attr.padding_r = 0;
    padding_attr.if_memset = 0;
    int num = 1;
    ret = (int)bmcv_image_vpp_basic(handle, 1, &src, dst, &num, loca, &padding_attr, resize_method, csc_type, NULL);
    return ret;
}

static const enum AVPixelFormat supported_in_formats[] = {
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ422P, // TODO HW VPP don't support J422 input
        AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_RGB24,
        AV_PIX_FMT_BGR24,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV444P
};
static const enum AVPixelFormat supported_out_formats[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_RGBP,
        AV_PIX_FMT_BGRP,
        AV_PIX_FMT_RGB24,
        AV_PIX_FMT_BGR24,
        AV_PIX_FMT_GRAY8
};

enum {
    BMSCALE_OPT_SCALE = 0,
    BMSCALE_OPT_CROP,
    BMSCALE_OPT_PAD
};

enum {
    BMSCALE_CSC_OTHER      = 0,
    BMSCALE_CSC_I420ToJ420 = 1,
    BMSCALE_CSC_NV12ToJ420 = 2,
    BMSCALE_CSC_J420ToI420 = 3,
    BMSCALE_CSC_J422ToJ420 = 4,

};

typedef struct BmScaleContext {
    const AVClass *class;

    enum AVPixelFormat in_fmt;
    enum AVPixelFormat out_fmt;

    struct {
        int width;
        int height;
    } in_planes[3], out_planes[3];

    AVBufferRef *frames_ctx;
    AVFrame     *frame;
    AVFrame     *black_frame; /* i420, j420, bgr24 */
    AVFrame     *tmp_frame;

    int          csc;           /* 0: none; 1: I420ToJ420; 2: J420ToI420; 3:J422ToJ420 */
    AVBufferRef *rgb_frames_ctx;
    AVFrame     *rgb_frame;     /* I420 --> scaled RGB --> J420 */

    AVBufferRef *yuv420p_frames_ctx;
    AVFrame     *yuv420p_frame; /* J422 --> J420 --> scaled J420 */

    int          passthrough;

    /**
     * New dimensions. Special values are:
     *   0 = original width/height
     *  -1 = keep original aspect
     */
    //int w, h;

    /**
     * Output sw format. AV_PIX_FMT_NONE for no conversion.
     */
    enum AVPixelFormat format;

    char*  w_expr;       /* width  expression string */
    char*  h_expr;       /* height expression string */
    char*  format_str;

    int    opt;
    int    flags;

    int    perf;        /* indicate if do the performance testing */
    double total_time;  /* ms */
    long   total_frame;

    int    b_debug;
    FILE*  outf;
    FILE*  blackf;

    bm_handle_t handle;
    /* add bmvpp support for normal pipeline */
    int soc_idx;
    int is_bmvpp_inited;
    int zero_copy;
    int stride_align;
    struct list_head freed_avbuffer_list;
    struct list_head module_entry;

} BmScaleContext;

/////////////////////////////////////////////////////////////////////////////////////////////////
// for scale_bm new version
#define BMVPP_MAX_CONVERT_STAGE_NUM 8
#define BMVPP_ALLOC(type, n) (type*)av_mallocz(sizeof(type)*(n))

static LIST_HEAD(g_module_list);

typedef int (*bmscale_csc_func_t)(int is_compressed, bm_image *s, bmcv_rect_t *crop, bm_image *d, int x, int y, int size_method);

struct bmscale_csc_stage_s {
    int format;
    bmscale_csc_func_t pfn_convert;
};

struct bmscale_csc_table_s {
    int src_format;
    int dst_format;
    int support_scale;
    int support_crop;
    int stage_num;
    struct bmscale_csc_stage_s stages[BMVPP_MAX_CONVERT_STAGE_NUM];
};

static int bmscale_csc_table_key_compare(const void* a1, const void* a2) {
    struct bmscale_csc_table_s *key1 = (struct bmscale_csc_table_s*)a1;
    struct bmscale_csc_table_s *key2 = (struct bmscale_csc_table_s*)a2;
    uint32_t ck1 = (uint16_t)key1->src_format << 16 | (uint16_t)key1->dst_format;
    uint32_t ck2 = (uint16_t)key2->src_format << 16 | (uint16_t)key2->dst_format;
    if (ck1 > ck2) return 1;
    if (ck1 == ck2) return 0;
    return -1;
}

struct bmscale_avbuffer_ctx_s {
    int format;
    int width;
    int height;
    bm_device_mem_t* ion_buff;
    uint8_t *vaddr;
    int soc_idx;
    int black_filled;
    bm_handle_t handle;
    BmScaleContext* ctx;
    struct list_head entry;
};

static int input_format_is_supported(enum AVPixelFormat fmt)
{
    int i;
    for (i = 0; i < FF_ARRAY_ELEMS(supported_in_formats); i++)
        if (supported_in_formats[i] == fmt)
            return 1;
    return 0;
}
static int output_format_is_supported(enum AVPixelFormat fmt)
{
    int i;
    for (i = 0; i < FF_ARRAY_ELEMS(supported_out_formats); i++)
        if (supported_out_formats[i] == fmt)
            return 1;
    return 0;
}

static int bmscale_csc_calulate(int in_fmt, int out_fmt)
{
    int csc = 0;
    if (in_fmt == AV_PIX_FMT_YUVJ422P && out_fmt == AV_PIX_FMT_YUVJ420P)
        csc = BMSCALE_CSC_J422ToJ420;
    else if (in_fmt == AV_PIX_FMT_YUVJ420P && out_fmt == AV_PIX_FMT_YUV420P)
        csc = BMSCALE_CSC_J420ToI420; /* MJPEG TO H.264 */
    else if (in_fmt == AV_PIX_FMT_YUV420P && out_fmt == AV_PIX_FMT_YUVJ420P)
        csc = BMSCALE_CSC_I420ToJ420;
    else if (in_fmt == AV_PIX_FMT_NV12 && out_fmt == AV_PIX_FMT_YUVJ420P)
        csc = BMSCALE_CSC_NV12ToJ420;
    else
        csc = BMSCALE_CSC_OTHER;
    return csc;
}

static int bmscale_get_auto_crop(BmScaleContext *ctx, AVFrame* src, AVFrame *dst, bmcv_rect_t *rc)
{
    int w0 = src->width;
    int h0 = src->height;
    int w1 = dst->width;
    int h1 = dst->height;
    bmcv_rect_t rect={0};

    if (w0 * h1 >= w1 * h0) {        /* w0/h0 > w1/h1 */
        int w2 = (w1 * h0 / h1 + 1) & (~1);
        int l0 = (w0 - w2);

        rect.start_x = (l0 / 2) & (~1);
        rect.start_y = 0;
        rect.crop_w = src->width - l0;
        rect.crop_h = src->height;;
        av_log(ctx, AV_LOG_TRACE, "crop: <x:%d, y:%d, width:%d, height:%d>\n",
               rect.start_x, rect.start_y, rect.crop_w, rect.crop_h);
    } else if (w0 * h1 < w1 * h0) { /* w0/h0 < w1/h1 */
        int h2 = (w0 * h1 / w1 + 1) & (~1);
        int l0 = (h0 - h2);

        rect.start_x = 0;
        rect.start_y = (l0 / 2) & (~1);
        rect.crop_w = src->width;
        rect.crop_h = src->height - l0;
        av_log(ctx, AV_LOG_TRACE, "crop: <x:%d, y:%d, width:%d, height:%d>\n",
               rect.start_x, rect.start_y, rect.crop_w, rect.crop_h);
    }

    *rc = rect;
    return 0;
}

static int bmscale_get_auto_pad(BmScaleContext *ctx, AVFrame* src, AVFrame *dst, int *p_top, int *p_bottom, int *p_left, int *p_right)
{
    int w0 = src->width;
    int h0 = src->height;
    int w1 = dst->width;
    int h1 = dst->height;
    int top=0, left=0, bottom=0, right=0;

    if (w0 * h1 > w1 * h0) {        /* w0/h0 > w1/h1 */
        int h2 = (w1 * h0 / w0 + 1) & (~1);
        int l0 = (h1 - h2);
        top = (l0 / 2) & (~1);
        bottom = l0 - top;
    } else if (w0 * h1 < w1 * h0) { /* w0/h0 < w1/h1 */
        int w2 = (w0 * h1 / h0 + 1) & (~1);
        int l0 = (w1 - w2);
        left = (l0 / 2) & (~1);
        right = l0 - left;
    }

    *p_top = top;
    *p_left = left;
    *p_right = right;
    *p_bottom = bottom;
    return 0;
}


/**
 * release refence buffer
 */

static void bmscale_avbuffer_release(struct bmscale_avbuffer_ctx_s *avbuffer_ctx)
{
    int soc_idx = 0;
    // list detach
    if (avbuffer_ctx->ctx != NULL) {
        list_del(&avbuffer_ctx->entry);
    }
    // do real free.
    soc_idx = avbuffer_ctx->soc_idx;
    if (avbuffer_ctx->ion_buff->size != 0) {
        bm_free_device(avbuffer_ctx->handle, *(avbuffer_ctx->ion_buff));
        bm_dev_free(avbuffer_ctx->handle);
    }

    // for pcie, system memory is allocated by av_malloc
    if (avbuffer_ctx->vaddr) {
        av_freep(&avbuffer_ctx->vaddr);
    }

    av_free(avbuffer_ctx);
}


static void bmscale_avbuffer_recycle(void *ctx, uint8_t *data)
{
    int found = 0;
    struct bmscale_avbuffer_ctx_s *avbuffer_ctx = (struct bmscale_avbuffer_ctx_s*)ctx;
    struct BmScaleContext* s;
    list_for_each_entry(s, &g_module_list, module_entry, struct BmScaleContext) {
        if (s == avbuffer_ctx->ctx) {
            found = 1;
            break;
        }
    }
    //recycle it, not really free.
    if (found) {
        list_add(&avbuffer_ctx->entry, &avbuffer_ctx->ctx->freed_avbuffer_list);
    }else{
        avbuffer_ctx->ctx = NULL;
        bmscale_avbuffer_release(avbuffer_ctx);
    }
}

static int bmscale_avimage_get_buffer_size(enum AVPixelFormat pix_fmt,
                             int width, int height, int align)
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

    return bmscale_avimage_fill_arrays(data, linesize, NULL, pix_fmt,
                                width, height, align);
}

static struct bmscale_avbuffer_ctx_s* bmscale_buffer_pool_alloc(BmScaleContext *ctx, int format, int width, int height)
{
    int ret = 0;
    int soc_idx = ctx->soc_idx;
    struct bmscale_avbuffer_ctx_s* hwpic = NULL;
    int found = 0;
    int total_buffer_size = 0;
    bm_handle_t handle;

    list_for_each_entry(hwpic, &ctx->freed_avbuffer_list, entry, struct bmscale_avbuffer_ctx_s) {
        if (hwpic->format == format && hwpic->width == width && hwpic->height == height) {
            found = 1;
            break;
        }
    }

    if (found) {
        // detach from free chain list.
        list_del(&hwpic->entry);
        return hwpic;
    }

    // don't found, try to create new
    hwpic = BMVPP_ALLOC(struct bmscale_avbuffer_ctx_s, 1);
    list_init(&hwpic->entry);
    hwpic->format = format;
    hwpic->width = width;
    hwpic->height = height;
    hwpic->soc_idx = ctx->soc_idx;
    hwpic->ctx = ctx;

    // allocate continuous memory from device,
    total_buffer_size = bmscale_avimage_get_buffer_size(format, width, height, ctx->stride_align);

    hwpic->ion_buff = (bm_device_mem_t *)av_mallocz(sizeof(bm_device_mem_t));

    //Handle is required for bmlib release,This is essential!!!
    bm_dev_request(&handle, hwpic->soc_idx);

    hwpic->handle = handle;

    ret = bmvpu_malloc_device_byte_heap(ctx, handle, hwpic->ion_buff, total_buffer_size, HEAP_MASK_1_2, 1);
    if(ret != BM_SUCCESS)
    {
        av_log(NULL, AV_LOG_ERROR, "[%s, %d] bmvpu_malloc_device_byte_heap() failed!\n", __FILE__, __LINE__);
        free(hwpic->ion_buff);
        av_free(hwpic);
        return NULL;
    }

        hwpic->vaddr = av_malloc(total_buffer_size);
        if (hwpic->vaddr == NULL) {
            av_log(ctx, AV_LOG_ERROR, "malloc err!\n");
            bm_free_device(ctx->handle, *(hwpic->ion_buff));
            av_free(hwpic->ion_buff);
            av_free(hwpic);
            return NULL;
        }

    return hwpic;
}

static int bmscale_init(AVFilterContext *ctx)
{
    BmScaleContext *s = ctx->priv;
    bm_handle_t handle;
    unsigned int chipid;
    int ret;

    av_log(s, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (!strcmp(s->format_str, "none")) {
        s->format = AV_PIX_FMT_NONE;
    } else {
        s->format = av_get_pix_fmt(s->format_str);
        if (!output_format_is_supported(s->format)) {
            av_log(s, AV_LOG_ERROR, "Unrecognized pixel format: %s\n", s->format_str);
            return AVERROR(EINVAL);
        }
    }

    if (s->opt == BMSCALE_OPT_SCALE)
        av_log(s, AV_LOG_DEBUG, "opt: scaling.\n");
    else if (s->opt == BMSCALE_OPT_CROP)
        av_log(s, AV_LOG_DEBUG, "opt: cropping.\n");
    else if (s->opt == BMSCALE_OPT_PAD)
        av_log(s, AV_LOG_DEBUG, "opt: padding.\n");

    if (s->flags == BMCV_INTER_BICUBIC)
        av_log(s, AV_LOG_DEBUG, "flags: bicubic.\n");
    else if (s->flags == BMCV_INTER_LINEAR)
        av_log(s, AV_LOG_DEBUG, "flags: bilinear.\n");
    else if (s->flags == BMCV_INTER_NEAREST)
        av_log(s, AV_LOG_DEBUG, "flags: nearest.\n");

    av_log(s, AV_LOG_DEBUG, "perf: %d\n", s->perf);
    if (s->perf) {
        s->total_time  = 0.0f;
        s->total_frame = 0L;
    }

    s->frame = av_frame_alloc();
    if (!s->frame)
        return AVERROR(ENOMEM);

    s->tmp_frame = av_frame_alloc();
    if (!s->tmp_frame)
        return AVERROR(ENOMEM);

    if (s->opt == BMSCALE_OPT_PAD) {
        s->black_frame = av_frame_alloc();
        if (!s->black_frame)
            return AVERROR(ENOMEM);
    } else {
        s->black_frame = NULL;
    }

    s->rgb_frame = av_frame_alloc();
    if (!s->rgb_frame)
        return AVERROR(ENOMEM);

    s->yuv420p_frame = av_frame_alloc();
    if (!s->yuv420p_frame)
        return AVERROR(ENOMEM);

    s->b_debug = 0;
    if (s->b_debug) {
        s->outf   = fopen("scale.yuv", "wb");
        s->blackf = fopen("black.yuv", "wb");
    }

    s->is_bmvpp_inited = 0;
    s->stride_align = 1;

    list_init(&s->freed_avbuffer_list);
    // qsort(g_convert_table, FF_ARRAY_ELEMS(g_convert_table), sizeof(g_convert_table[0]), bmscale_csc_table_key_compare);

    list_init(&s->module_entry);
    list_add(&g_module_list, &s->module_entry);

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;
}

static void bmscale_uninit(AVFilterContext *ctx)
{
    BmScaleContext *s = ctx->priv;

    av_log(s, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (s->perf) {
        if (s->total_time > 0.0f) {
            av_log(s, AV_LOG_INFO, "Frames filtered: %ld. filtering speed: %.0ffps\n",
                   s->total_frame, s->total_frame*1000/s->total_time);
        }
    }

    list_del(&s->module_entry);

    if (s->is_bmvpp_inited) {
        bm_dev_free(s->handle);
        s->is_bmvpp_inited = 0;
    }

    {
        struct bmscale_avbuffer_ctx_s *pos, *n;
        list_for_each_entry_safe(pos, n, &s->freed_avbuffer_list, entry, struct bmscale_avbuffer_ctx_s) {
            bmscale_avbuffer_release(pos);
        }
    }

    av_frame_free(&s->frame);
    av_frame_free(&s->tmp_frame);
    av_frame_free(&s->black_frame);
    av_buffer_unref(&s->frames_ctx);

    av_frame_free(&s->rgb_frame);
    av_buffer_unref(&s->rgb_frames_ctx);

    av_frame_free(&s->yuv420p_frame);
    av_buffer_unref(&s->yuv420p_frames_ctx);

    if (s->b_debug) {
        if (s->outf)
            fclose(s->outf);
        if (s->blackf)
            fclose(s->blackf);
    }

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
}

static int bmscale_query_formats_hwaccel(AVFilterContext *ctx)
{
    BmScaleContext *s = ctx->priv;
    static const enum AVPixelFormat pixel_formats[] = {
            AV_PIX_FMT_BMCODEC, AV_PIX_FMT_NONE,
    };
    AVFilterFormats *pix_fmts;
    int ret;

    av_log(s, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    pix_fmts = ff_make_format_list(pixel_formats);

    ret = ff_set_common_formats(ctx, pix_fmts);

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return ret;
}

static int bmscale_query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats;
    enum AVPixelFormat pix_fmt;
    int ret;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (ctx->hw_device_ctx != NULL) {
        return bmscale_query_formats_hwaccel(ctx);
    }


    if (ctx->inputs[0]) {
        const AVPixFmtDescriptor *desc = NULL;
        formats = NULL;
        while ((desc = av_pix_fmt_desc_next(desc))) {
            pix_fmt = av_pix_fmt_desc_get_id(desc);
            if ((input_format_is_supported(pix_fmt))
                && (ret = ff_add_format(&formats, pix_fmt)) < 0) {
                return ret;
            }
        }
        if ((ret = ff_formats_ref(formats, &ctx->inputs[0]->outcfg.formats)) < 0)
            return ret;
    }
    if (ctx->outputs[0]) {
        const AVPixFmtDescriptor *desc = NULL;
        formats = NULL;
        while ((desc = av_pix_fmt_desc_next(desc))) {
            pix_fmt = av_pix_fmt_desc_get_id(desc);
            if ((output_format_is_supported(pix_fmt))
                && (ret = ff_add_format(&formats, pix_fmt)) < 0) {
                return ret;
            }
        }
        if ((ret = ff_formats_ref(formats, &ctx->outputs[0]->incfg.formats)) < 0)
            return ret;
    }

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;

}


static int init_black_picture(BmScaleContext *s, AVFrame* black)
{
    AVHWFramesContext* frm_ctx = (AVHWFramesContext*)black->hw_frames_ctx->data;
    AVBmCodecFrame*      hwpic = (AVBmCodecFrame*)black->data[4];
    bm_device_mem_t         *pb = (bm_device_mem_t *)hwpic->buffer;
    uint8_t *p, *c;
    int size0, size1, total_size;
    int ret;

    if (frm_ctx->sw_format == AV_PIX_FMT_BGR24) {
        total_size = size0 = pb->size;
        size1 = 0;
    } else {
        size0 = hwpic->data[1] - hwpic->data[0];
        size1 = hwpic->data[2] - hwpic->data[1];
        total_size = size0 + size1 + size1;
    }

    p = av_malloc(total_size);
    if (p == NULL)
        return AVERROR(ENOMEM);
    c = p + size0;

    if (frm_ctx->sw_format == AV_PIX_FMT_YUVJ420P) {
        memset(p,    0, size0);
        memset(c, 0x80, size1+size1);
    } else if (frm_ctx->sw_format == AV_PIX_FMT_YUV420P) {
        memset(p,   16, size0);
        memset(c, 0x80, size1+size1);
    } else /* if (frm_ctx->sw_format == AV_PIX_FMT_BGR24) */ {
        memset(p,    0, total_size);
    }

    if(!s->handle)
        s->handle = hwpic->handle;
    ret = bm_memcpy_s2d(s->handle, *pb, p);
    if ( ret != BM_SUCCESS) {
        av_log(s, AV_LOG_ERROR, "bm_memcpy_s2d failed\n");
        return AVERROR_EXTERNAL;
    }

    av_free(p);

    return 0;
}

static int init_hw_out_contexts(BmScaleContext *s, AVBufferRef *device_ctx)
{
    AVBufferRef *out_ref     = NULL;
    AVBufferRef *rgb_ref     = NULL;
    AVBufferRef *yuv420p_ref = NULL;
    AVHWFramesContext *frames_ctx;
    AVHWFramesContext *rgb_frames_ctx;
    AVHWFramesContext *yuv420p_frames_ctx;
    int in_sw, in_sh, out_sw, out_sh;
    int ret, i;

    s->csc = bmscale_csc_calulate(s->in_fmt, s->out_fmt);
    s->frames_ctx = NULL;
    s->rgb_frames_ctx = NULL;
    s->yuv420p_frames_ctx = NULL;

    av_pix_fmt_get_chroma_sub_sample(s->in_fmt,  &in_sw,  &in_sh);
    av_pix_fmt_get_chroma_sub_sample(s->out_fmt, &out_sw, &out_sh);

    if (!s->out_planes[0].width) {
        s->out_planes[0].width  = s->in_planes[0].width;
        s->out_planes[0].height = s->in_planes[0].height;
    }
    for (i = 1; i < FF_ARRAY_ELEMS(s->in_planes); i++) {
        s->in_planes[i].width   = s->in_planes[0].width   >> in_sw;
        s->in_planes[i].height  = s->in_planes[0].height  >> in_sh;
        s->out_planes[i].width  = s->out_planes[0].width  >> out_sw;
        s->out_planes[i].height = s->out_planes[0].height >> out_sh;
    }

    for (i = 0; i < FF_ARRAY_ELEMS(s->in_planes); i++) {
        av_log(s, AV_LOG_DEBUG, "in_planes[%d]: %dx%d\n",
               i, s->in_planes[i].width, s->in_planes[i].height);
    }
    for (i = 0; i < FF_ARRAY_ELEMS(s->out_planes); i++) {
        av_log(s, AV_LOG_DEBUG, "out_planes[%d]: %dx%d\n", i,
               s->out_planes[i].width, s->out_planes[i].height);
    }

    out_ref = av_hwframe_ctx_alloc(device_ctx);
    if (!out_ref)
        return AVERROR(ENOMEM);

    frames_ctx = (AVHWFramesContext*)out_ref->data;

    frames_ctx->format    = AV_PIX_FMT_BMCODEC;
    frames_ctx->sw_format = s->out_fmt;
    frames_ctx->width     = FFALIGN(s->out_planes[0].width,  32);
    frames_ctx->height    = FFALIGN(s->out_planes[0].height, 32);
    // TODO from the input parameter
    frames_ctx->initial_pool_size = 0; // 64 + s->extra_hw_frames;

    ret = av_hwframe_ctx_init(out_ref);
    if (ret < 0)
        goto fail;

    av_log(s, AV_LOG_DEBUG, "out_frames_ctx->format=%s\n",
           (char*)av_x_if_null(av_get_pix_fmt_name(frames_ctx->format), "invalid"));
    av_log(s, AV_LOG_DEBUG, "out_frames_ctx->sw_format=%s\n",
           (char*)av_x_if_null(av_get_pix_fmt_name(frames_ctx->sw_format), "invalid"));
    av_log(s, AV_LOG_DEBUG, "out_frames_ctx->width =%d\n", frames_ctx->width);
    av_log(s, AV_LOG_DEBUG, "out_frames_ctx->height=%d\n", frames_ctx->height);

    av_frame_unref(s->frame);
    ret = av_hwframe_get_buffer(out_ref, s->frame, 0);
    if (ret < 0)
        goto fail;

    s->frame->width  = s->out_planes[0].width;
    s->frame->height = s->out_planes[0].height;

    av_buffer_unref(&s->frames_ctx);
    s->frames_ctx = out_ref;

    /**
     * for rgb frame
     */

    if (s->csc == BMSCALE_CSC_J422ToJ420) {
        yuv420p_ref = av_hwframe_ctx_alloc(device_ctx);
        if (!yuv420p_ref)
            return AVERROR(ENOMEM);

        yuv420p_frames_ctx = (AVHWFramesContext*)yuv420p_ref->data;

        yuv420p_frames_ctx->format    = AV_PIX_FMT_BMCODEC;
        yuv420p_frames_ctx->sw_format = AV_PIX_FMT_YUVJ420P;
        yuv420p_frames_ctx->width     = FFALIGN(s->in_planes[0].width,  32);
        yuv420p_frames_ctx->height    = FFALIGN(s->in_planes[0].height, 32);
        yuv420p_frames_ctx->initial_pool_size = 0;

        ret = av_hwframe_ctx_init(yuv420p_ref);
        if (ret < 0)
            goto fail;

        av_log(s, AV_LOG_DEBUG, "yuv420p_frames_ctx->format=%s\n",
               (char*)av_x_if_null(av_get_pix_fmt_name(yuv420p_frames_ctx->format), "invalid"));
        av_log(s, AV_LOG_DEBUG, "yuv420p_frames_ctx->sw_format=%s\n",
               (char*)av_x_if_null(av_get_pix_fmt_name(yuv420p_frames_ctx->sw_format), "invalid"));
        av_log(s, AV_LOG_DEBUG, "yuv420p_frames_ctx->width =%d\n", yuv420p_frames_ctx->width);
        av_log(s, AV_LOG_DEBUG, "yuv420p_frames_ctx->height=%d\n", yuv420p_frames_ctx->height);

        av_frame_unref(s->yuv420p_frame);
        ret = av_hwframe_get_buffer(yuv420p_ref, s->yuv420p_frame, 0);
        if (ret < 0)
            goto fail;

        s->yuv420p_frame->width  = s->in_planes[0].width;
        s->yuv420p_frame->height = s->in_planes[0].height;

        av_buffer_unref(&s->yuv420p_frames_ctx);
        s->yuv420p_frames_ctx = yuv420p_ref;
    } else if (s->csc == BMSCALE_CSC_I420ToJ420 ||
               s->csc == BMSCALE_CSC_NV12ToJ420 ||
               s->csc == BMSCALE_CSC_J420ToI420) {
        rgb_ref = av_hwframe_ctx_alloc(device_ctx);
        if (!rgb_ref)
            return AVERROR(ENOMEM);

        rgb_frames_ctx = (AVHWFramesContext*)rgb_ref->data;

        rgb_frames_ctx->format    = AV_PIX_FMT_BMCODEC;
        rgb_frames_ctx->sw_format = AV_PIX_FMT_BGR24;
        rgb_frames_ctx->width     = FFALIGN(s->out_planes[0].width,  32);
        rgb_frames_ctx->height    = FFALIGN(s->out_planes[0].height, 32);
        rgb_frames_ctx->initial_pool_size = 0;

        ret = av_hwframe_ctx_init(rgb_ref);
        if (ret < 0)
            goto fail;

        av_log(s, AV_LOG_DEBUG, "rgb_frames_ctx->format=%s\n",
               (char*)av_x_if_null(av_get_pix_fmt_name(rgb_frames_ctx->format), "invalid"));
        av_log(s, AV_LOG_DEBUG, "rgb_frames_ctx->sw_format=%s\n",
               (char*)av_x_if_null(av_get_pix_fmt_name(rgb_frames_ctx->sw_format), "invalid"));
        av_log(s, AV_LOG_DEBUG, "rgb_frames_ctx->width =%d\n", rgb_frames_ctx->width);
        av_log(s, AV_LOG_DEBUG, "rgb_frames_ctx->height=%d\n", rgb_frames_ctx->height);

        av_frame_unref(s->rgb_frame);
        ret = av_hwframe_get_buffer(rgb_ref, s->rgb_frame, 0);
        if (ret < 0)
            goto fail;

        s->rgb_frame->width  = s->out_planes[0].width;
        s->rgb_frame->height = s->out_planes[0].height;

        av_buffer_unref(&s->rgb_frames_ctx);
        s->rgb_frames_ctx = rgb_ref;
    }

    if (s->opt == BMSCALE_OPT_PAD) {
        if (s->csc == BMSCALE_CSC_I420ToJ420 ||
            s->csc == BMSCALE_CSC_NV12ToJ420 ||
            s->csc == BMSCALE_CSC_J420ToI420)
            ret = av_hwframe_get_buffer(s->rgb_frame->hw_frames_ctx, s->black_frame, 0);
        else
            ret = av_hwframe_get_buffer(s->frame->hw_frames_ctx, s->black_frame, 0);
        if (ret < 0) {
            goto fail;
        }

        ret = init_black_picture(s, s->black_frame);
        if (ret < 0)
            goto fail;
    }


    return 0;
fail:
    av_buffer_unref(&out_ref);
    av_buffer_unref(&rgb_ref);
    av_buffer_unref(&yuv420p_ref);
    return ret;
}

static int bmscale_config_props_hwaccel(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    BmScaleContext    *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    int in_w = inlink->w;
    int in_h = inlink->h;
    int out_w, out_h;
    enum AVPixelFormat in_format;
    enum AVPixelFormat out_format;
    int ret;

    av_log(s, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    ret = ff_scale_eval_dimensions(s, s->w_expr, s->h_expr, inlink, outlink, &out_w, &out_h);
    if (ret < 0)
        goto fail;

    av_log(s, AV_LOG_DEBUG, "out_w=%d\n", out_w);
    av_log(s, AV_LOG_DEBUG, "out_h=%d\n", out_h);

    if (((int64_t)out_h * in_w) > INT_MAX  || ((int64_t)out_w * in_h) > INT_MAX) {
        av_log(s, AV_LOG_ERROR, "Rescaled value for width or height is too big.\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }

    /* check that we have a hw context */
    if (!inlink->hw_frames_ctx) {
        av_log(s, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }

    in_frames_ctx = (AVHWFramesContext*)inlink->hw_frames_ctx->data;

    av_log(s, AV_LOG_DEBUG, "in_frames_ctx->format=%s\n",
           (char*)av_x_if_null(av_get_pix_fmt_name(in_frames_ctx->format), "invalid"));
    av_log(s, AV_LOG_DEBUG, "in_frames_ctx->sw_format=%s\n",
           (char*)av_x_if_null(av_get_pix_fmt_name(in_frames_ctx->sw_format), "invalid"));
    av_log(s, AV_LOG_DEBUG, "in_frames_ctx->width =%d\n", in_frames_ctx->width);
    av_log(s, AV_LOG_DEBUG, "in_frames_ctx->height=%d\n", in_frames_ctx->height);

    in_format = in_frames_ctx->sw_format;
    if (!input_format_is_supported(in_format)) {
        av_log(s, AV_LOG_ERROR, "Unsupported input format: %s\n",
               av_get_pix_fmt_name(in_format));
        return AVERROR(ENOSYS);
    }

    if (s->format == AV_PIX_FMT_NONE) {
        if (in_format == AV_PIX_FMT_NV12) {
            av_log(s, AV_LOG_WARNING, "Specified output format: yuv420p\n");
            out_format = AV_PIX_FMT_YUV420P;
        } else {
            out_format = in_format;
        }
    } else {
        out_format = s->format;
    }
    if (!output_format_is_supported(out_format)) {
        av_log(s, AV_LOG_ERROR, "Unsupported output format: %s\n",
               av_get_pix_fmt_name(out_format));
        return AVERROR(ENOSYS);
    }

    if (in_w == out_w && in_h == out_h && in_format == out_format) {
        s->passthrough = 1;
        av_log(s, AV_LOG_INFO, "work in passthrough mode.\n");
    }

    s->in_fmt               = in_format;
    s->out_fmt              = out_format;
    s->in_planes[0].width   = in_w;
    s->in_planes[0].height  = in_h;
    s->out_planes[0].width  = out_w;
    s->out_planes[0].height = out_h;

    av_log(s, AV_LOG_INFO, "in_fmt: %s; out_fmt: %s\n",
           av_get_pix_fmt_name(s->in_fmt), av_get_pix_fmt_name(s->out_fmt));

    /* init the hardware output contexts */
    ret = init_hw_out_contexts(s, in_frames_ctx->device_ref);
    if (ret < 0)
        return ret;

    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(s->frames_ctx);
    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);

    av_log(s, AV_LOG_VERBOSE, "w:%d h:%d -> w:%d h:%d\n",
           in_w, in_h, out_w, out_h);

    if (inlink->sample_aspect_ratio.num)
        outlink->sample_aspect_ratio = av_mul_q((AVRational){out_h*in_w, out_w*in_h},
                                                inlink->sample_aspect_ratio);
    else
        outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;

    outlink->w = out_w;
    outlink->h = out_h;

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
    return 0;

fail:
    return ret;
}

static int bmscale_config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    BmScaleContext    *s = ctx->priv;
    int in_w = inlink->w;
    int in_h = inlink->h;

    int out_w, out_h;
    int in_format, out_format;
    int ret;

    if (ctx->hw_device_ctx != NULL) {
        return bmscale_config_props_hwaccel(outlink);
    }

    ret = ff_scale_eval_dimensions(s, s->w_expr, s->h_expr,inlink, outlink, &out_w, &out_h);
    if (ret != 0) {
        goto fail;
    }

    out_w = out_w & (~1);
    out_h = out_h & (~1);

    if (((int64_t)out_h * out_w) > INT_MAX || ((int64_t)out_w*out_h) > INT_MAX) {
        av_log(s, AV_LOG_ERROR, "Rescaled value for width or height is too big.\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }

    in_format = inlink->format;
    av_log(s, AV_LOG_INFO, "filter suggested:output format: %s\n",
           av_get_pix_fmt_name(outlink->format));

    if (s->format != AV_PIX_FMT_NONE) {
        out_format = s->format;
    }else {
        out_format = outlink->format;
    }

    if (!output_format_is_supported(out_format)) {
        av_log(s, AV_LOG_ERROR, "Unsupported output format: %s\n",
               av_get_pix_fmt_name(out_format));
        return AVERROR(ENOSYS);
    }

    av_log(s, AV_LOG_INFO, "in_format:%s ->> out_format:%s\n", av_get_pix_fmt_name(in_format),
            av_get_pix_fmt_name(out_format));

    if (in_w == out_w && in_h == out_h && in_format == out_format) {
        s->passthrough = 1;
        av_log(s, AV_LOG_INFO, "work in pass through mode\n");
    }

    s->csc = bmscale_csc_calulate(in_format, out_format);
    if (!s->is_bmvpp_inited) {
        ret = bm_dev_request(&s->handle, s->soc_idx);
        if (ret != BM_SUCCESS){
            av_log(s, AV_LOG_ERROR, "bm_dev_request(%d) err=%d\n", s->soc_idx, ret);
            goto fail;
        }
        s->is_bmvpp_inited = 1;
    }

    if (inlink->sample_aspect_ratio.num)
        outlink->sample_aspect_ratio = av_mul_q((AVRational){out_h*in_w, out_w*in_h},
                inlink->sample_aspect_ratio);
    else
        outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;

    outlink->w = out_w;
    outlink->h = out_h;
    outlink->format = out_format;

    return 0;

fail:
    bm_dev_free(s->handle);
    return ret;
}

static int download_data(AVFrame* frame, FILE* outf)
{
    AVBmCodecFrame* hwpic = (AVBmCodecFrame*)frame->data[4];
    uint8_t* yuv;
    int size[2];
    int total_size;
    int ret = 0;

    if (hwpic==NULL || hwpic->buffer==NULL|| hwpic->maptype==1) {
        return 0;
    }

    size[0] = hwpic->data[1] - hwpic->data[0];
    size[1] = hwpic->data[2] - hwpic->data[1];
    total_size = size[0] + size[1] + size[1];

    yuv = malloc(total_size);

    av_log(NULL, AV_LOG_DEBUG, "bm_memcpy_d2s begin: buffer = %p\n", hwpic->buffer);
    av_log(NULL, AV_LOG_TRACE, "data 0: %p\n", hwpic->data[0]);
    av_log(NULL, AV_LOG_TRACE, "data 1: %p\n", hwpic->data[1]);
    av_log(NULL, AV_LOG_TRACE, "data 2: %p\n", hwpic->data[2]);

    av_log(NULL, AV_LOG_TRACE, "line size 0: %d\n", hwpic->linesize[0]);
    av_log(NULL, AV_LOG_TRACE, "line size 1: %d\n", hwpic->linesize[1]);
    av_log(NULL, AV_LOG_TRACE, "line size 2: %d\n", hwpic->linesize[2]);

    av_log(NULL, AV_LOG_TRACE, "width : %d\n", frame->width);
    av_log(NULL, AV_LOG_TRACE, "height: %d\n", frame->height);

    ret = bm_memcpy_d2s(hwpic->handle, yuv, *((bm_device_mem_t*)hwpic->buffer));
    if (ret != 0) {
        av_log(NULL, AV_LOG_ERROR, "bm_memcpy_d2s failed\n");
        goto Exit;
    }

    av_log(NULL, AV_LOG_DEBUG, "bm_ion_download_data end\n");

    fwrite(yuv, sizeof(uint8_t), total_size, outf);
    fflush(outf);

    Exit:
    free(yuv);

    return ret;
}

static bm_image_format_ext get_format_from_avformat(int sw_format){
    if (sw_format == AV_PIX_FMT_YUV420P || sw_format == AV_PIX_FMT_YUVJ420P)
        return FORMAT_YUV420P;

    if (sw_format == AV_PIX_FMT_YUV422P || sw_format == AV_PIX_FMT_YUVJ422P)
        return FORMAT_YUV422P; // TODO

    if (sw_format == AV_PIX_FMT_YUVJ444P || sw_format == AV_PIX_FMT_YUV444P)
        return FORMAT_YUV444P; // TODO

    if (sw_format == AV_PIX_FMT_RGB24)
        return FORMAT_RGB_PACKED;

    if (sw_format == AV_PIX_FMT_BGR24)
        return FORMAT_BGR_PACKED;

    if (sw_format == AV_PIX_FMT_RGBP)
        return FORMAT_RGBP_SEPARATE;

    if (sw_format == AV_PIX_FMT_BGRP)
        return FORMAT_BGRP_SEPARATE;

    if (sw_format == AV_PIX_FMT_NV12)
        return FORMAT_NV12;

    av_log(NULL, AV_LOG_ERROR, "avformat %d is not supported!\n", sw_format);
    return -1;
}

static int bmscale_hw_bmimg_from_avframe(bm_handle_t handle, AVHWFramesContext* ctx, AVBmCodecFrame* src, AVFrame* avimg, bm_image* image, int is_compressed){
    bm_image_format_ext format = get_format_from_avformat(ctx->sw_format);
    int stride[4] = {src->linesize[0], src->linesize[1], src->linesize[2], src->linesize[3]};
    uint64_t pa[4]={0};
    int size[4] = {0};
    if(is_compressed){
        pa[0]   = (uint64_t)src->data[2]; // y table
        pa[1]   = (uint64_t)src->data[0]; // y base
        pa[2]   = (uint64_t)src->data[3]; // c table
        pa[3]   = (uint64_t)src->data[1]; // c base
        size[0] = src->linesize[2];
        size[1] = src->linesize[0];
        size[2] = src->linesize[3];
        size[3] = src->linesize[1];
        bm_image_create(handle, avimg->height, avimg->width, FORMAT_COMPRESSED, DATA_TYPE_EXT_1N_BYTE, image, stride);
    }
    else{
        pa[0]   = (uint64_t)src->data[0];
        pa[1]   = (uint64_t)src->data[1];
        pa[2]   = (uint64_t)src->data[2];
        switch (format)
        {
        case FORMAT_YUV420P:
            size[2] = stride[2] * avimg->height / 2;
        case FORMAT_NV12:
            size[1] = stride[1] * avimg->height / 2;
            size[0] = stride[0] * avimg->height;
            break;
        case FORMAT_YUV422P:
        case FORMAT_YUV444P:
        case FORMAT_RGBP_SEPARATE:
        case FORMAT_BGRP_SEPARATE:
            size[1] = stride[1] * avimg->height;
            size[2] = stride[2] * avimg->height;
        case FORMAT_RGB_PACKED:
        case FORMAT_BGR_PACKED:
            size[0] = stride[0] * avimg->height;
            break;
        default:
            av_log(NULL, AV_LOG_ERROR, "bmformat %d is not supported!\n", format);
            return -1;
        }
        bm_image_create(handle, avimg->height, avimg->width, format, DATA_TYPE_EXT_1N_BYTE, image, stride);
    }
    bm_device_mem_t mem[4]={0};
    for(int i = 0; i < 4; i++){
        bm_set_device_mem(&mem[i], size[i], pa[i]);
    }
    bm_image_attach(image[0], mem);
    return 0;
}

static int bmscale_copy(BmScaleContext* ctx, AVFrame* dst, AVFrame* src)
{
    AVHWFramesContext* src_frm_ctx = (AVHWFramesContext*)src->hw_frames_ctx->data;
    AVHWFramesContext* dst_frm_ctx = (AVHWFramesContext*)dst->hw_frames_ctx->data;
    AVBmCodecFrame*          hwsrc = (AVBmCodecFrame*)src->data[4];
    AVBmCodecFrame*          hwdst = (AVBmCodecFrame*)dst->data[4];
    bm_device_mem_t*    ion_buffer = (bm_device_mem_t*)hwdst->buffer;
    bm_device_mem_t*    src_buffer = (bm_device_mem_t*)hwsrc->buffer;
    bm_handle_t handle;
    int dev_id = ctx->soc_idx;
    int ret = bm_dev_request(&handle, dev_id);
    if (ret != BM_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Create bm handle failed. ret = %d\n", ret);
        return ret;
    }
    av_log(ctx, AV_LOG_TRACE, "soc_idx %d\n", ctx->soc_idx);
    bm_image bmsrc, bmdst;
    bmcv_rect_t crop;
    if (src_frm_ctx->sw_format != dst_frm_ctx->sw_format ||
        src_frm_ctx->format != AV_PIX_FMT_BMCODEC ||
        dst_frm_ctx->format != AV_PIX_FMT_BMCODEC) {
        return AVERROR(EINVAL);
    }

    if (hwsrc->linesize[0] != hwdst->linesize[0] ||
        hwsrc->linesize[1] != hwdst->linesize[1] ||
        hwsrc->linesize[2] != hwdst->linesize[2]) {
        return AVERROR(EINVAL);
    }

    if (hwdst->padded) {
        av_log(ctx, AV_LOG_TRACE, "recycled frame\n");
        return 0;
    }
    av_log(ctx, AV_LOG_TRACE, "src sw format: %s\n", av_get_pix_fmt_name(src_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "src data 0: %p\n", hwsrc->data[0]);
    av_log(ctx, AV_LOG_TRACE, "src data 1: %p\n", hwsrc->data[1]);
    av_log(ctx, AV_LOG_TRACE, "src data 2: %p\n", hwsrc->data[2]);

    av_log(ctx, AV_LOG_TRACE, "line size 0: %d\n", hwsrc->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "line size 1: %d\n", hwsrc->linesize[1]);
    av_log(ctx, AV_LOG_TRACE, "line size 2: %d\n", hwsrc->linesize[2]);

    av_log(ctx, AV_LOG_TRACE, "src width : %d\n", src->width);
    av_log(ctx, AV_LOG_TRACE, "src height: %d\n", src->height);

    av_log(ctx, AV_LOG_TRACE, "dst sw format: %s\n", av_get_pix_fmt_name(dst_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "dst data 0: %p\n", hwdst->data[0]);
    av_log(ctx, AV_LOG_TRACE, "dst data 1: %p\n", hwdst->data[1]);
    av_log(ctx, AV_LOG_TRACE, "dst data 2: %p\n", hwdst->data[2]);

    av_log(ctx, AV_LOG_TRACE, "dst width : %d\n", dst->width);
    av_log(ctx, AV_LOG_TRACE, "dst height: %d\n", dst->height);

    ret = bmscale_hw_bmimg_from_avframe(handle, src_frm_ctx, hwsrc, src, &bmsrc, 0);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    ret = bmscale_hw_bmimg_from_avframe(handle, dst_frm_ctx, hwdst, dst, &bmdst, 0);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    crop.start_x = 0;
    crop.start_y = 0;
    crop.crop_w = bmsrc.width;
    crop.crop_h = bmsrc.height;
    csc_type_t csc_type;
    ret = bmcv_get_csc_type_by_colorinfo(src->colorspace, src->color_range, bmsrc.data_type, bmdst.data_type, &csc_type);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    ret = bmvpp_scale_bmcv(handle, bmsrc, &crop, &bmdst, 0, 0, 0, 0, ctx->flags, csc_type);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    hwdst->padded = 1;
fail:
    bm_image_destroy(&bmsrc);
    bm_image_destroy(&bmdst);
    if(handle != NULL){
        bm_dev_free(handle);
    }
    if(ret != BM_SUCCESS){
        av_log(ctx, AV_LOG_ERROR, "bmvpp_copy failed\n");
    }
    return ret;
}

static int bmscale_J422ToJ420_hwaccel(BmScaleContext* ctx, AVFrame* dst, AVFrame* src)
{
    AVHWFramesContext* src_frm_ctx = (AVHWFramesContext*)src->hw_frames_ctx->data;
    AVHWFramesContext* dst_frm_ctx = (AVHWFramesContext*)dst->hw_frames_ctx->data;
    AVBmCodecFrame*          hwsrc = (AVBmCodecFrame*)src->data[4];
    AVBmCodecFrame*          hwdst = (AVBmCodecFrame*)dst->data[4];
    bm_device_mem_t*    ion_buffer = (bm_device_mem_t*)hwdst->buffer;
    bm_device_mem_t*    src_buffer = (bm_device_mem_t*)hwsrc->buffer;
    bm_handle_t handle;
    int dev_id = ctx->soc_idx;
    int ret = bm_dev_request(&handle, dev_id);
    if (ret != BM_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Create bm handle failed. ret = %d\n", ret);
        return ret;
    }
    av_log(ctx, AV_LOG_TRACE, "soc_idx %d\n", ctx->soc_idx);
    bm_image bmsrc, bmdst;
    bmcv_rect_t crop;
    if (src_frm_ctx->format != AV_PIX_FMT_BMCODEC ||
        dst_frm_ctx->format != AV_PIX_FMT_BMCODEC) {
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_TRACE, "src sw format: %s\n", av_get_pix_fmt_name(src_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "src color space: %s\n", av_color_space_name(src->colorspace));
    av_log(ctx, AV_LOG_TRACE, "src data 0: %p\n", hwsrc->data[0]);
    av_log(ctx, AV_LOG_TRACE, "src data 1: %p\n", hwsrc->data[1]);
    av_log(ctx, AV_LOG_TRACE, "src data 2: %p\n", hwsrc->data[2]);

    av_log(ctx, AV_LOG_TRACE, "src line size 0: %d\n", hwsrc->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "src line size 1: %d\n", hwsrc->linesize[1]);
    av_log(ctx, AV_LOG_TRACE, "src line size 2: %d\n", hwsrc->linesize[2]);

    av_log(ctx, AV_LOG_TRACE, "src width : %d\n", src->width);
    av_log(ctx, AV_LOG_TRACE, "src height: %d\n", src->height);

    av_log(ctx, AV_LOG_TRACE, "dst sw format: %s\n", av_get_pix_fmt_name(dst_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "dst color space: %s\n", av_color_space_name(dst->colorspace));
    av_log(ctx, AV_LOG_TRACE, "dst data 0: %p\n", hwdst->data[0]);
    av_log(ctx, AV_LOG_TRACE, "dst data 1: %p\n", hwdst->data[1]);
    av_log(ctx, AV_LOG_TRACE, "dst data 2: %p\n", hwdst->data[2]);

    av_log(ctx, AV_LOG_TRACE, "dst line size 0: %d\n", hwdst->linesize[0]);
    av_log(ctx, AV_LOG_TRACE, "dst line size 1: %d\n", hwdst->linesize[1]);
    av_log(ctx, AV_LOG_TRACE, "dst line size 2: %d\n", hwdst->linesize[2]);

    av_log(ctx, AV_LOG_TRACE, "dst width : %d\n", dst->width);
    av_log(ctx, AV_LOG_TRACE, "dst height: %d\n", dst->height);

    ret = bmscale_hw_bmimg_from_avframe(handle, src_frm_ctx, hwsrc, src, &bmsrc, 0);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    ret = bmscale_hw_bmimg_from_avframe(handle, dst_frm_ctx, hwdst, dst, &bmdst, 0);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    crop.start_x = 0;
    crop.start_y = 0;
    crop.crop_w = bmsrc.width;
    crop.crop_h = bmsrc.height;
    csc_type_t csc_type;
    ret = bmcv_get_csc_type_by_colorinfo(src->colorspace, src->color_range, bmsrc.data_type, bmdst.data_type, &csc_type);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    ret = bmvpp_scale_bmcv(handle, bmsrc, &crop, &bmdst, 0, 0, 0, 0, ctx->flags, csc_type);
    if(ret != BM_SUCCESS){
        goto fail;
    }
fail:
    bm_image_destroy(&bmsrc);
    bm_image_destroy(&bmdst);
    if(handle != NULL){
        bm_dev_free(handle);
    }
    if(ret != BM_SUCCESS){
        av_log(ctx, AV_LOG_ERROR, "bmscale_J422ToJ420_hwaccel failed\n");
    }
    return ret;
}

static int bmscale_filtering_hwaccel(BmScaleContext* ctx, int opt, AVFrame* dst, AVFrame* src)
{
    AVHWFramesContext* src_frm_ctx = (AVHWFramesContext*)src->hw_frames_ctx->data;
    AVHWFramesContext* dst_frm_ctx = (AVHWFramesContext*)dst->hw_frames_ctx->data;
    AVBmCodecFrame*          hwsrc = (AVBmCodecFrame*)src->data[4];
    AVBmCodecFrame*          hwdst = (AVBmCodecFrame*)dst->data[4];

    bm_device_mem_t*    ion_buffer = (bm_device_mem_t*)hwdst->buffer;
    bm_device_mem_t*    src_buffer = (bm_device_mem_t*)hwsrc->buffer;

    int                    maptype =  hwsrc->maptype;

    bm_handle_t handle;
    int dev_id = ctx->soc_idx;
    int ret = bm_dev_request(&handle, dev_id);
    if (ret != BM_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Create bm handle failed. ret = %d\n", ret);
        return ret;
    }
    av_log(ctx, AV_LOG_TRACE, "soc_idx %d\n", dev_id);
    bm_image bmsrc, bmdst;
    bmcv_rect_t crop_rect;
    int i;
    if (src_frm_ctx->format != AV_PIX_FMT_BMCODEC ||
        dst_frm_ctx->format != AV_PIX_FMT_BMCODEC) {
        return AVERROR(EINVAL);
    }
    av_log(ctx, AV_LOG_TRACE, "src sw format: %s\n", av_get_pix_fmt_name(src_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "src color space: %s\n", av_color_space_name(src->colorspace));
    for (i=0; i<4; i++)
        av_log(ctx, AV_LOG_TRACE, "src data %d: %p\n", i, hwsrc->data[i]);
    for (i=0; i<4; i++)
        av_log(ctx, AV_LOG_TRACE, "src line size %d: %d\n", i, hwsrc->linesize[i]);

    av_log(ctx, AV_LOG_TRACE, "src width : %d\n", src->width);
    av_log(ctx, AV_LOG_TRACE, "src height: %d\n", src->height);

    av_log(ctx, AV_LOG_TRACE, "dst sw format: %s\n", av_get_pix_fmt_name(dst_frm_ctx->sw_format));
    av_log(ctx, AV_LOG_TRACE, "dst color space: %s\n", av_color_space_name(dst->colorspace));
    for (i=0; i<4; i++)
        av_log(ctx, AV_LOG_TRACE, "dst data %d: %p\n", i, hwdst->data[i]);
    for (i=0; i<4; i++)
        av_log(ctx, AV_LOG_TRACE, "dst line size %d: %d\n", i, hwdst->linesize[i]);

    av_log(ctx, AV_LOG_TRACE, "dst width : %d\n", dst->width);
    av_log(ctx, AV_LOG_TRACE, "dst height: %d\n", dst->height);

    bm_image_data_format_ext s_data_type = get_format_from_avformat(src_frm_ctx->sw_format);
    int s_width    = hwsrc->coded_width > 0 ? hwsrc->coded_width:src->width;
    int s_height   = hwsrc->coded_height > 0 ? hwsrc->coded_height:src->height;
    crop_rect.start_x = 0;
    crop_rect.start_y = 0;
    crop_rect.crop_w = src->width;
    crop_rect.crop_h = src->height;
    int s_stride[4]={hwsrc->linesize[0], hwsrc->linesize[1], hwsrc->linesize[2], hwsrc->linesize[3]};
    uint64_t s_pa[4]={0};
    int s_size[4]={0};

    ret = bmscale_hw_bmimg_from_avframe(handle, src_frm_ctx, hwsrc, src, &bmsrc, maptype);
    if(ret != BM_SUCCESS){
        goto fail;
    }
    ret = bmscale_hw_bmimg_from_avframe(handle, dst_frm_ctx, hwdst, dst, &bmdst, 0);
    if(ret != BM_SUCCESS){
        goto fail;
    }

    csc_type_t csc_type;
    ret = bmcv_get_csc_type_by_colorinfo(src->colorspace, src->color_range, bmsrc.data_type, bmdst.data_type, &csc_type);
    if(ret != BM_SUCCESS){
        goto fail;
    }

    if (opt == BMSCALE_OPT_CROP) {
        bmcv_rect_t rect;
        bmscale_get_auto_crop(ctx, src, dst, &rect);
        ret = bmvpp_scale_bmcv(handle, bmsrc, &rect, &bmdst, 0, 0, 0, 0, ctx->flags, csc_type);
    } else if (opt == BMSCALE_OPT_PAD) {
        int top = 0, bottom = 0, left = 0, right = 0;
        bmscale_get_auto_pad(ctx, src, dst, &top, &bottom, &left, &right);
        av_log(ctx, AV_LOG_TRACE,
               "pad: <left:%d, right:%d, top:%d, bottom:%d>, width:%d, height:%d\n",
               left, right, top, bottom, bmdst.width-left-right, bmdst.height-top-bottom);
        ret = bmvpp_scale_bmcv(handle, bmsrc, &crop_rect, &bmdst, left, top, right, bottom, ctx->flags, csc_type);
    } else {
        ret = bmvpp_scale_bmcv(handle, bmsrc, &crop_rect, &bmdst, 0, 0, 0, 0, ctx->flags, csc_type);
    }
fail:
    bm_image_destroy(&bmsrc);
    bm_image_destroy(&bmdst);
    if(handle != NULL){
        bm_dev_free(handle);
    }
    if(ret != BM_SUCCESS){
        av_log(ctx, AV_LOG_ERROR, "bmscale_filtering_hwaccel failed\n");
    }
    return ret;
}

static int bmscale_filter_frame_hwaccel(AVFilterLink *link, AVFrame *in_frame)
{
    AVFilterContext      *ctx = link->dst;
    BmScaleContext        *s  = ctx->priv;
    AVFilterLink     *outlink = ctx->outputs[0];
    //AVHWFramesContext     *frames_ctx = (AVHWFramesContext*)outlink->hw_frames_ctx->data;
    //AVBmCodecDeviceContext *device_hwctx = frames_ctx->device_ctx->hwctx; // TODO
    AVBmCodecFrame*          hwsrc = (AVBmCodecFrame*)in_frame->data[4];
    struct timeval ps, pe;

    AVFrame *out_frame = NULL;
    int ret = 0;

    av_log(s, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);
    av_log(s, AV_LOG_TRACE, "maptype=%d\n", hwsrc->maptype);
    av_log(s, AV_LOG_TRACE, "width =%d\n", in_frame->width);
    av_log(s, AV_LOG_TRACE, "height=%d\n", in_frame->height);
    av_log(s, AV_LOG_TRACE, "s->frame->width =%d\n", s->frame->width);
    av_log(s, AV_LOG_TRACE, "s->frame->height=%d\n", s->frame->height);

    if (s->passthrough && hwsrc->maptype==0)
        return ff_filter_frame(outlink, in_frame);

    if (s->perf) {
        gettimeofday(&ps, NULL);
    }

    out_frame = av_frame_alloc();
    if (!out_frame) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    /* hw scaling in_frame to s->frame */
    if (s->csc == BMSCALE_CSC_J422ToJ420) {
        if (s->opt == BMSCALE_OPT_PAD) {
            ret = bmscale_copy(s, s->frame, s->black_frame);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_copy failed\n", __func__, __LINE__);
                goto fail;
            }
            if (s->b_debug)
                download_data(s->frame, s->blackf);
        }

        /* J422 ---> J420 */
        ret = bmscale_J422ToJ420_hwaccel(s, s->yuv420p_frame, in_frame);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
            goto fail;
        }

        /* J420 --> scaled J420 */
        ret = bmscale_filtering_hwaccel(s, s->opt, s->frame, s->yuv420p_frame);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
            goto fail;
        }
    } else if (s->csc == BMSCALE_CSC_I420ToJ420 ||
               s->csc == BMSCALE_CSC_NV12ToJ420 ||
               s->csc == BMSCALE_CSC_J420ToI420) {
        if (s->opt == BMSCALE_OPT_PAD) {
            ret = bmscale_copy(s, s->rgb_frame, s->black_frame);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_copy failed\n", __func__, __LINE__);
                goto fail;
            }
        }

        /* I420/J420 --> scaled BGR24 */
        ret = bmscale_filtering_hwaccel(s, s->opt, s->rgb_frame, in_frame);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
            goto fail;
        }

        /* BGR24 --> J420/I420 */
        ret = bmscale_filtering_hwaccel(s, BMSCALE_OPT_PAD, s->frame, s->rgb_frame);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
            goto fail;
        }
    } else {
        if (s->opt == BMSCALE_OPT_PAD) {
            ret = bmscale_copy(s, s->frame, s->black_frame);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_copy failed\n", __func__, __LINE__);
                goto fail;
            }
            if (s->b_debug)
                download_data(s->frame, s->blackf);
        }
        ret = bmscale_filtering_hwaccel(s, s->opt, s->frame, in_frame);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
            goto fail;
        }
    }

    if (s->b_debug) {
        ret = download_data(s->frame, s->outf);
        if (ret != 0) {
            av_log(s, AV_LOG_ERROR, "download_data failed\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    ret = av_hwframe_get_buffer(s->frame->hw_frames_ctx, s->tmp_frame, 0);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "[%s,%d] av_hwframe_get_buffer failed\n", __func__, __LINE__);
        goto fail;
    }

    s->tmp_frame->width  = s->out_planes[0].width;
    s->tmp_frame->height = s->out_planes[0].height;

    av_frame_move_ref(out_frame, s->frame);
    av_frame_move_ref(s->frame, s->tmp_frame);

    ret = av_frame_copy_props(out_frame, in_frame);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "[%s,%d] av_frame_copy_props failed\n", __func__, __LINE__);
        goto fail;
    }

    av_reduce(&out_frame->sample_aspect_ratio.num, &out_frame->sample_aspect_ratio.den,
              (int64_t)in_frame->sample_aspect_ratio.num * outlink->h * link->w,
              (int64_t)in_frame->sample_aspect_ratio.den * outlink->w * link->h,
              INT_MAX);

    av_frame_free(&in_frame);

    if (s->perf) {
        gettimeofday(&pe, NULL);
        s->total_time += ((pe.tv_sec*1000.0 + pe.tv_usec/1000.0) -
                          (ps.tv_sec*1000.0 + ps.tv_usec/1000.0));
        s->total_frame++;
    }

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
    return ff_filter_frame(outlink, out_frame);
fail:
    av_frame_free(&in_frame);
    av_frame_free(&out_frame);
    return ret;
}


static void fill_black_frame(AVFrame * frame)
{
    if (frame->format == AV_PIX_FMT_YUVJ420P ) {
        memset(frame->data[0], 0,  frame->linesize[0] * frame->height);
        memset(frame->data[1], 0x80, frame->linesize[1] * (frame->height >>1));
        memset(frame->data[2], 0x80, frame->linesize[2] * (frame->height >>1));
    } else if (frame->format == AV_PIX_FMT_YUV420P) {
        memset(frame->data[0], 16,  frame->linesize[0] * frame->height);
        memset(frame->data[1], 0x80, frame->linesize[1] * (frame->height >>1));
        memset(frame->data[2], 0x80, frame->linesize[2] * (frame->height >>1));
    } else if (frame->format == AV_PIX_FMT_BGR24 || frame->format == AV_PIX_FMT_RGB24 ||
               frame->format == AV_PIX_FMT_ABGR || frame->format == AV_PIX_FMT_ARGB) {
        memset(frame->data[0], 0,  frame->linesize[0] * frame->height);
    }else if (frame->format == AV_PIX_FMT_BGRP || frame->format == AV_PIX_FMT_RGBP ||
              frame->format == AV_PIX_FMT_YUV444P) {
        memset(frame->data[0], 0,  frame->linesize[0] * frame->height);
        memset(frame->data[1], 0, frame->linesize[1] * frame->height);
        memset(frame->data[2], 0, frame->linesize[2] * frame->height);
    }else if (frame->format == AV_PIX_FMT_GRAY8) {
        memset(frame->data[0], 0, frame->linesize[0] * frame->height);
    }else if (frame->format == AV_PIX_FMT_YUVJ422P || frame->format == AV_PIX_FMT_YUV422P) {
        memset(frame->data[0], 0, frame->linesize[0] * frame->height);
        memset(frame->data[1], 0x80, frame->linesize[1] * frame->height);
        memset(frame->data[2], 0x80, frame->linesize[2] * frame->height);
    }
    else {
        av_log(NULL, AV_LOG_ERROR, "ERROR:fill_back_frame() not support format=%s\n", av_get_pix_fmt_name(frame->format));
    }
}

static int bmscale_avimage_fill_arrays(uint8_t *dst_data[4], int dst_linesize[4],
                         const uint8_t *src, enum AVPixelFormat pix_fmt,
                         int width, int height, int align)
{
    int ret, i;

    ret = av_image_check_size(width, height, 0, NULL);
    if (ret < 0)
        return ret;

    ret = av_image_fill_linesizes(dst_linesize, pix_fmt, width);
    if (ret < 0)
        return ret;

    switch (pix_fmt) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUVJ422P:
        dst_linesize[1] = FFALIGN(dst_linesize[1], align);
        dst_linesize[0] = dst_linesize[1] * 2;
        dst_linesize[2] = dst_linesize[1];
        break;
    default:
        for (i = 0; i < 4; i++)
            dst_linesize[i] = FFALIGN(dst_linesize[i], align);
        break;
    }

    return av_image_fill_pointers(dst_data, pix_fmt, height, (uint8_t *)src, dst_linesize);
}

static int bmscale_avframe_get_buffer(BmScaleContext *ctx, AVFrame* frame, int opt)
{
    int ret = 0;
    int total_buffer_size = 0;

    struct bmscale_avbuffer_ctx_s *hwpic = bmscale_buffer_pool_alloc(ctx, frame->format, frame->width, frame->height);
    if (NULL == hwpic) return -1;
    // fill system memory pointers
    bmscale_avimage_fill_arrays(frame->data, frame->linesize, hwpic->vaddr, frame->format, frame->width, frame->height, ctx->stride_align);
    bmscale_avimage_fill_arrays(&frame->data[4], &frame->linesize[4], (void*)hwpic->ion_buff->u.device.device_addr, frame->format, frame->width, frame->height, ctx->stride_align);

    // fill black background,it's important.
    if (BMSCALE_OPT_PAD == opt && hwpic->black_filled == 0) {
        fill_black_frame(frame);
        // upload data to device memory.
        ret = bm_memcpy_s2d(ctx->handle, *(hwpic->ion_buff), hwpic->vaddr);
        //ret = bm_ion_upload_data(hwpic->vaddr, hwpic->ion_buff, hwpic->ion_buff->size);
        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR, "[%s, %d] bm_memcpy_s2d failed!\n", __FILE__, __LINE__);
        }

        hwpic->black_filled = 1;
    }

    frame->buf[0] = av_buffer_create(frame->data[0], total_buffer_size, bmscale_avbuffer_recycle, hwpic, 0);
    if (frame->buf[0] == NULL) {
        ret = AVERROR(ENOMEM);
        goto failed;
    }

    return 0;
failed:
    bmscale_avbuffer_release(hwpic);
    return ret;
}

static int bmscale_avframe_download(BmScaleContext *ctx, AVFrame *frame)
{
    int ret = 0;
    AVBufferRef *ref = frame->buf[0];
    BmScaleContext *ctx_tmp = ctx;

    if (ref != NULL){
        struct bmscale_avbuffer_ctx_s *ctx=(struct bmscale_avbuffer_ctx_s*)av_buffer_get_opaque(ref);
        if (ctx->vaddr != ctx->ion_buff->u.system.system_addr) {
            //d2s
            ret = bm_memcpy_d2s(ctx_tmp->handle, ctx->vaddr, *(ctx->ion_buff));
            if (ret != 0) {
                av_log(ctx_tmp, AV_LOG_ERROR, "bm_memcpy_d2s error! 1787\n");
                return ret;
            }
        }else{
            //TODO
        }
    }
    return ret;
}

static int bmscale_bmimg_from_avframe(bm_handle_t handle, BmScaleContext *ctx, AVFrame *src, bm_image *bmsrc, int *is_compressed)
{
    bm_image_format_ext format = get_format_from_avformat(src->format);
    int num_comp;
    // vppmat->color_range = src->color_range;
    // vppmat->colorspace  = src->colorspace;
    int width    = src->width;
    int height   = src->height;
    int stride[4]={src->linesize[0], src->linesize[1], src->linesize[2], src->linesize[3]};
    uint64_t pa[4]={0};
    int size[4]={0};
    // vppmat->soc_idx  = ctx->soc_idx;
    bm_device_mem_t src_mem[4]={0};
    if (src->data[4] != NULL) {
        // this avframe has physical address
        if (src->format == AV_PIX_FMT_NV12) {
            if (101 == src->channel_layout) {
                AVBmCodecFrame* hwpic;
                if (src->buf[4] && src->buf[4]->size == sizeof(AVBmCodecFrame)){
                    hwpic = (AVBmCodecFrame*)src->buf[4]->data;
                    width = hwpic->coded_width;
                    height = hwpic->coded_height;
                } else {
                    av_log(ctx, AV_LOG_ERROR, "avFrame buffer[4] should not be null at compression mode!\n");
                    return -1;
                }

                if (is_compressed) *is_compressed = 1;
                num_comp = 2;
                /* compressed format */
                pa[0] = (uint64_t) src->data[2 + 4]; // y table
                pa[1] = (uint64_t) src->data[0 + 4]; // y base
                pa[2] = (uint64_t) src->data[3 + 4]; // c table
                pa[3] = (uint64_t) src->data[1 + 4]; // c base

                size[0] = src->linesize[6]; // luma table size
                size[1] = src->linesize[4] * hwpic->coded_height; // y base size
                size[2] = src->linesize[7]; // chroma table size
                size[3] = src->linesize[5] * hwpic->coded_height/2; //chroma base size

            } else {
                pa[0] = (uint64_t) src->data[4]; // y
                pa[1] = (uint64_t) src->data[5]; // cb cr
                size[0] = src->linesize[4] * src->height; //Y
                size[1] = src->linesize[5] * src->height/2; //UV
            }
        }else if (AV_PIX_FMT_YUV420P == src->format || AV_PIX_FMT_YUVJ420P == src->format) {
            num_comp = 3;
            pa[0] = (uint64_t) src->data[4]; // y
            pa[1] = (uint64_t) src->data[5]; // cb
            pa[2] = (uint64_t) src->data[6]; // cr

            size[0] = src->linesize[4] * src->height; //Y
            size[1] = src->linesize[5] * src->height / 2; //U
            size[2] = src->linesize[6] * src->height / 2; //V
        }else if (AV_PIX_FMT_RGBP == src->format || AV_PIX_FMT_BGRP == src->format || AV_PIX_FMT_YUV444P == src->format) {
            num_comp = 3;
            pa[0] = (uint64_t) src->data[4]; // R/B
            pa[1] = (uint64_t) src->data[5]; // G
            pa[2] = (uint64_t) src->data[6]; // B/R

            size[0] =  src->linesize[4] * src->height;
            size[1] =  src->linesize[5] * src->height;
            size[2] =  src->linesize[6] * src->height;
        } else if (AV_PIX_FMT_RGB24 == src->format || AV_PIX_FMT_BGR24 == src->format) {
            num_comp = 3;
            pa[0] = (uint64_t) src->data[4]; // RGB24
            size[0] =  src->linesize[4] * src->height;
        } else if (AV_PIX_FMT_GRAY8 == src->format) {
            num_comp = 1;
            pa[0] = (uint64_t) src->data[4]; // Y only
            size[0] =  src->linesize[4] * src->height;
        }else if (AV_PIX_FMT_YUVJ422P == src->format || AV_PIX_FMT_YUV422P == src->format) {
            num_comp = 1;
            pa[0] = (uint64_t) src->data[4]; // Y
            size[0] =  src->linesize[4] * src->height;

            pa[1] = (uint64_t) src->data[5]; // U
            size[1] =  src->linesize[5] * src->height;

            pa[2] = (uint64_t) src->data[6]; // V
            size[2] =  src->linesize[6] * src->height;
        }
        else {
            av_log(ctx, AV_LOG_ERROR, "Don't support format=%s, when direct use physical address!\n",
                   av_get_pix_fmt_name(src->format));
            return -1;
        }
        if(is_compressed){
            if(*is_compressed){
                bm_image_create(handle, height, width, FORMAT_COMPRESSED, DATA_TYPE_EXT_1N_BYTE, bmsrc, stride);}
            else{
                bm_image_create(handle, height, width, format, DATA_TYPE_EXT_1N_BYTE, bmsrc, stride);}
        }
        else{
            bm_image_create(handle, height, width, format, DATA_TYPE_EXT_1N_BYTE, bmsrc, stride);
        }
        for(int i = 0; i < 4; i++){
            bm_set_device_mem(&src_mem[i], size[i], pa[i]);
        }
        bm_image_attach(bmsrc[0], src_mem);
    }
    else {
        if (is_compressed) *is_compressed = 0;
        bm_image_create(handle, height, width, format, DATA_TYPE_EXT_1N_BYTE, bmsrc, stride);
        // frame on normal system memory, alloc memory from ion for color convertion.
        if (bm_image_alloc_dev_mem_heap_mask(bmsrc[0], HEAP_MASK_1_2) != BM_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "bm_image_alloc() err!\n");
            return -1;
        }
        void* in_ptr[4] = {(void*)src->data[0], (void*)src->data[1], (void*)src->data[2], (void*)src->data[3]};
        // copy data from avframe to vpp
        if(bm_image_copy_host_to_device(bmsrc[0], (void **)in_ptr) != BM_SUCCESS){
            av_log(ctx, AV_LOG_ERROR, "bm_image_copy_host_to_device() err!\n");
            return -1;
        }
    }
        //TODO
    return 0;
}

static int bmscale_is_support(struct bmscale_csc_table_s *item, int src_format, int dst_format, int need_scaled, int opt)
{
    if (item != NULL)
    {
        int supported = 1;
        int need_crop = (opt== BMSCALE_OPT_PAD || opt == BMSCALE_OPT_CROP);
        if (item->support_crop == 0 && need_crop) {
            supported = 0;
        }
        if (item->support_scale == 0 && need_scaled) {
            supported = 0;
        }
        return supported;
    }

    return 0;
}

static int bmscale_filtering(BmScaleContext* ctx, int opt, AVFrame *src, AVFrame *dst)
{
    bm_handle_t handle = NULL;
    int dev_id = ctx->soc_idx;
    int ret = bm_dev_request(&handle, dev_id);
    if (ret != BM_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Create bm handle failed. ret = %d\n", ret);
        return ret;
    }
    int is_compressed_nv12 = 0;
    ret = bmscale_avframe_get_buffer(ctx, dst, opt);
    if (ret != BM_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "bmscale_avframe_get_buffer() failed!\n");
        if(handle != NULL){
            bm_dev_free(handle);
        }
        return ret;
    }
    bm_image bmcv_s;
    bm_image bmcv_d;
    memset(&bmcv_s, 0, sizeof(bm_image));
    memset(&bmcv_d, 0, sizeof(bm_image));
    ret = bmscale_bmimg_from_avframe(handle, ctx, src, &bmcv_s, &is_compressed_nv12);
    if (ret != BM_SUCCESS) {
        goto fail;
    }
    ret = bmscale_bmimg_from_avframe(handle, ctx, dst, &bmcv_d, NULL);
    if (ret != BM_SUCCESS) {
        goto fail;
    }
    bmcv_rect_t rect;
    rect.start_x = 0;
    rect.start_y = 0;
    rect.crop_w = src->width;
    rect.crop_h = src->height;
    int scale_done = 0;
    csc_type_t csc_type;
    ret = bmcv_get_csc_type_by_colorinfo(src->colorspace, src->color_range, bmcv_s.data_type, bmcv_d.data_type, &csc_type);
    if (ret != BM_SUCCESS) {
        goto fail;
    }
    if (BMSCALE_OPT_SCALE == opt && !scale_done) {
        ret = bmvpp_scale_bmcv(handle, bmcv_s, &rect, &bmcv_d, 0, 0, 0, 0, ctx->flags, csc_type);
    }else if (BMSCALE_OPT_CROP == opt) {
        bmcv_rect_t rect_crop;
        bmscale_get_auto_crop(ctx, src, dst, &rect_crop);
        ret = bmvpp_scale_bmcv(handle, bmcv_s, &rect_crop, &bmcv_d, 0, 0, 0, 0, ctx->flags, csc_type);
    }else if (BMSCALE_OPT_PAD == opt) {
        int top, bottom, left, right;
        bmscale_get_auto_pad(ctx, src, dst, &top, &bottom, &left, &right);
        ret = bmvpp_scale_bmcv(handle, bmcv_s, &rect, &bmcv_d, left, top, right, bottom, ctx->flags, csc_type);
    }
fail:
    bm_image_destroy(&bmcv_s);
    bm_image_destroy(&bmcv_d);
    if(handle != NULL){
        bm_dev_free(handle);
    }
    if(ret != BM_SUCCESS){
        av_log(ctx, AV_LOG_ERROR, "bmscale_filtering failed!\n");
        return ret;
    }
    return (ctx->zero_copy ? 0:bmscale_avframe_download(ctx, dst));
}

static int bmscale_filter_frame(AVFilterLink *link, AVFrame *in_frame)
{
    AVFilterContext *ctx = link->dst;
    BmScaleContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out_frame=NULL;
    int ret = 0;

    if (ctx->hw_device_ctx != NULL) {
        return bmscale_filter_frame_hwaccel(link, in_frame);
    }

    if (s->passthrough) {
        //return ff_filter_frame(outlink, in_frame);
    }
    out_frame = av_frame_alloc();
    if (NULL == out_frame) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    out_frame->format = outlink->format;
    out_frame->width = outlink->w;
    out_frame->height = outlink->h;

    /* scale/padding/crop operating */
    ret = bmscale_filtering(s, s->opt, in_frame, out_frame);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "[%s,%d] bmscale_filtering failed\n", __func__, __LINE__);
        goto fail;
    }

    ret = av_frame_copy_props(out_frame, in_frame);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "[%s,%d] av_frame_copy_props failed\n", __func__, __LINE__);
        goto fail;
    }

    av_reduce(&out_frame->sample_aspect_ratio.num, &out_frame->sample_aspect_ratio.den,
              (int64_t)in_frame->sample_aspect_ratio.num * outlink->h * link->w,
              (int64_t)in_frame->sample_aspect_ratio.den * outlink->w * link->h,
              INT_MAX);

    av_frame_free(&in_frame);

    av_log(s, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);
    return ff_filter_frame(outlink, out_frame);
fail:
    av_frame_free(&in_frame);
    av_frame_free(&out_frame);
    return ret;
}

#define OFFSET(x) offsetof(BmScaleContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption bmscale_options[] = {
        { "w",      "Output video width",  OFFSET(w_expr),     AV_OPT_TYPE_STRING, { .str = "iw"   }, .flags = FLAGS },
        { "h",      "Output video height", OFFSET(h_expr),     AV_OPT_TYPE_STRING, { .str = "ih"   }, .flags = FLAGS },
        { "format", "Output pixel format", OFFSET(format_str), AV_OPT_TYPE_STRING, { .str = "none" }, .flags = FLAGS },
        { "perf", "indicate if profile the performance", OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
        { "opt", "resize operation", OFFSET(opt), AV_OPT_TYPE_INT,   { .i64 = BMSCALE_OPT_SCALE }, 0, 2, FLAGS, "opt" },
        { "crop",    "auto cropping",      0, AV_OPT_TYPE_CONST, { .i64 = BMSCALE_OPT_CROP  }, 0, 0, FLAGS, "opt" },
        { "pad",     "auto padding",       0, AV_OPT_TYPE_CONST, { .i64 = BMSCALE_OPT_PAD   }, 0, 0, FLAGS, "opt" },
        { "flags", "resize method", OFFSET(flags), AV_OPT_TYPE_INT,  { .i64 = BMCV_INTER_LINEAR  }, 0, 2, FLAGS, "flags" },
        { "bilinear","bilinear method",    0, AV_OPT_TYPE_CONST, { .i64 = BMCV_INTER_LINEAR }, 0, 0, FLAGS, "flags" },
        { "nearest", "nearest method",     0, AV_OPT_TYPE_CONST, { .i64 = BMCV_INTER_NEAREST  }, 0, 0, FLAGS, "flags" },
        { "bicubic", "bicubic method",     0, AV_OPT_TYPE_CONST, { .i64 = BMCV_INTER_BICUBIC  }, 0, 0, FLAGS, "flags" },
        { "sophon_idx", "sophon device index when running in pcie mode", OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 256, FLAGS },
        { "zero_copy", "flag to indicate if the output frame buffer only has physical buffer",
                OFFSET(zero_copy), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
        { NULL },
};

static const AVClass bmscale_class = {
        .class_name = "bmscale",
        .item_name  = av_default_item_name,
        .option     = bmscale_options,
        .version    = LIBAVUTIL_VERSION_INT,
        .category   = AV_CLASS_CATEGORY_FILTER,
};

static const AVFilterPad bmscale_inputs[] = {
        {
                .name         = "default",
                .type         = AVMEDIA_TYPE_VIDEO,
                .filter_frame = bmscale_filter_frame,
        },
};

static const AVFilterPad bmscale_outputs[] = {
        {
                .name             = "default",
                .type             = AVMEDIA_TYPE_VIDEO,
                .config_props     = bmscale_config_props,
        },
};

AVFilter ff_vf_scale_bm = {
        .name           = "scale_bm",
        .description    = NULL_IF_CONFIG_SMALL("Bitman scaling and format conversion"),

        .init           = bmscale_init,
        .uninit         = bmscale_uninit,
        FILTER_QUERY_FUNC(bmscale_query_formats),
        .priv_size      = sizeof(BmScaleContext),
        .priv_class     = &bmscale_class,
        FILTER_INPUTS(bmscale_inputs),
        FILTER_OUTPUTS(bmscale_outputs),

        .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE, // TODO
};
