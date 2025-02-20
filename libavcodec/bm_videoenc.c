/*
 * BM H.264/H.265 encoder
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
 *  BM H.264/H.265 encoder
 *
 * @author
 *  Solan Shang <shulin.shang@bitmain.com>
 *
 * Unimplemented:
 *   - video_full_range_flag
 *   - XXX
 */

#include <sys/time.h>

#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/decode.h"
#include "libavcodec/encode.h"
#include "libavcodec/packet_internal.h"
#if defined(BM1684)
#include "libavcodec/hwaccels.h"
#include "libavcodec/bm_codec.h"
#include "libavcodec/hwconfig.h"
#endif
#include "codec_internal.h"
#include "internal.h"
#include "bm_vpuenc_interface.h"
#include "bmlib_runtime.h"
#include "config_components.h"


#ifndef MIN
#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#endif

#define BM_MAX_CTU_NUM                     0x4000      // CTU num for max resolution = 8192x8192/(64x64)
#define BM_MAX_SUB_CTU_NUM                 (MAX_CTU_NUM*4)
#define BM_MAX_MB_NUM                      0x40000     // MB num for max resolution = 8192x8192/(16x16)
#define BM_VPU_ALIGN16(_x)             (((_x)+0x0f)&~0x0f)
#define BM_VPU_ALIGN64(_x)             (((_x)+0x3f)&~0x3f)

#define HEAP_MASK_1_2                      0x06
#define GET_STREAM_TIMEOUT  30*1000

#define ENC_PKT_DATA_SIZE 1024*1024

typedef struct {
    AVCodecContext *avctx;
    AVPacket* avpkt;
} bs_buffer_t;

typedef struct {
    const AVClass *avclass;
    AVCodecContext *avctx;

    // TODO
    AVBufferRef    *hw_device_ctx; /* pointer to HW accelerator (decoder) */
    AVBufferRef    *hw_frames_ctx; /* pointer to HW accelerator (frame allocator) */

    BmVpuEncOpenParams open_params;
    BmVpuEncoder* video_encoder;

    BmVpuEncInitialInfo initial_info;

    bm_device_mem_t *dma_memory_allocator;


    BmVpuFramebuffer* src_fb_list;
    BmVpuEncDMABuffer**  src_fb_dmabuffers;
    void*             frame_unused_queue;
    int num_src_fb;
    BmVpuFramebuffer* src_fb;

    /* For roi info*/
    BmCustomMapOpt* roi_list;
    BmVpuEncDMABuffer**  roi_dmabuffers;
    void*             roi_unused_queue;
    int num_roi;
    BmCustomMapOpt* roi_info;


    void*   avframe_used_list;

    BmVpuEncDMABuffer *bs_dma_buffer;
    size_t bs_buffer_size;
    unsigned int bs_buffer_alignment;

    bs_buffer_t bs_buffer;

    BmVpuEncParams enc_params;
    BmVpuRawFrame input_frame;
    BmVpuEncodedFrame output_frame;

    struct timeval ps;   /* encoding start time */
    struct timeval pe;   /* encoding end time */


    int    hw_accel;

    bool   is_end;        /* encoding end */

    int    zero_copy;     /* indicate if the input frame buffer is DMA buffer */

    int    soc_idx;       /* The index of PCIE Sophon device */
    int    core_idx;

    int    preset;        /* 0, 1, 2 */

    int    gop_preset;    /* 1 - 8 */

    int    cqp;

    int    roi_enable;    /*0 roi disable,  1 roi enable*/

    int    perf;          /* indicate if do the performance testing */
    char*  params_str;

    double total_time; // ms
    long   total_frame;
    int    first_pkt_recevie_flag;  // 1: have recv first pkt
    int    enc_cmd_queue; // venc command queue depth (1 ~ 4)
} BmVpuEncContext;

typedef union {
    struct {
        unsigned int  ctu_force_mode  :  2; //[ 1: 0]
        unsigned int  ctu_coeff_drop  :  1; //[    2]
        unsigned int  reserved        :  5; //[ 7: 3]
        unsigned int  sub_ctu_qp_0    :  6; //[13: 8]
        unsigned int  sub_ctu_qp_1    :  6; //[19:14]
        unsigned int  sub_ctu_qp_2    :  6; //[25:20]
        unsigned int  sub_ctu_qp_3    :  6; //[31:26]

        unsigned int  lambda_sad_0    :  8; //[39:32]
        unsigned int  lambda_sad_1    :  8; //[47:40]
        unsigned int  lambda_sad_2    :  8; //[55:48]
        unsigned int  lambda_sad_3    :  8; //[63:56]
    } field;
} EncCustomMap; // for wave5xx custom map (1 CTU = 64bits)

typedef union {
    struct {
        unsigned char  mb_force_mode  :  2; //lint !e46 [ 1: 0]
        unsigned char  mb_qp          :  6; //lint !e46 [ 7: 2]
    } field;
} AvcEncCustomMap; // for AVC custom map on wave  (1 MB = 8bits)


typedef struct {
    uint8_t* buffer;

    size_t   nmemb;
    size_t   size; /* bytes each element */
    int      count;

    int      front;
    int      rear;
} bm_queue_t;

static bm_queue_t* bm_queue_create(size_t nmemb, size_t size);
static void bm_queue_destroy(bm_queue_t* q);
static bool bm_queue_push(bm_queue_t* q, void* data);
static void* bm_queue_pop(bm_queue_t* q);
static bool bm_queue_is_full(bm_queue_t* q);
static bool bm_queue_is_empty(bm_queue_t* q);
static bool bm_queue_show(bm_queue_t* q);

static bm_queue_t* bm_queue_create(size_t nmemb, size_t size)
{
    bm_queue_t* q = NULL;
    q = (bm_queue_t *)malloc(sizeof(bm_queue_t));
    if (q == NULL)
        return NULL;

    q->nmemb  = nmemb;
    q->size   = size;
    q->count  = 0;
    q->front  = 0;
    q->rear   = 0;
    q->buffer = (uint8_t*)calloc(nmemb, size);

    return q;
}

static void bm_queue_destroy(bm_queue_t* q)
{
    if (q == NULL)
        return;

    if (q->buffer)
        free(q->buffer);

    free(q);
}

static bool bm_queue_push(bm_queue_t* q, void* data)
{
    uint8_t* ptr;
    int      offset;

    if (q == NULL)
        return false;

    if (data  == NULL)
        return false;

    /* check if queue is full */
    if (bm_queue_is_full(q))
        return false;

    offset = q->rear * q->size;
    ptr = &q->buffer[offset];
    memcpy(ptr, data, q->size);

    q->rear++;
    q->rear %= q->nmemb;

    q->count++;

    return true;
}

static void* bm_queue_pop(bm_queue_t* q)
{
    void* data;
    int   offset;

    if (q == NULL)
        return NULL;

    /* check if queue is empty */
    if (bm_queue_is_empty(q))
    {
        av_log(NULL, AV_LOG_DEBUG, "queue is empty\n");
        return NULL;
    }

    offset = q->front * q->size;
    data   = (void*)&q->buffer[offset];

    q->front++;
    q->front %= q->nmemb;

    q->count--;

    return data;
}

static bool bm_queue_is_full(bm_queue_t* q)
{
    bool ret;
    if (q == NULL)
        return false;

    ret = (q->count >= q->nmemb);

    return ret;
}

static bool bm_queue_is_empty(bm_queue_t* q)
{
    bool ret;
    if (q == NULL)
        return false;

    ret = (q->count <= 0);

    return ret;
}

static bool bm_queue_show(bm_queue_t* q)
{
    int start;

    if (q == NULL)
        return false;

    if (bm_queue_is_empty(q))
        return true;

    start = q->front;
    do
    {
        int offset   = start * q->size;
        int* ptr = (int*)(&q->buffer[offset]);

        printf("%s:%d(%s) count=%d. front=%d, rear=%d. %dth: 0x%08x\n",
               __FILE__, __LINE__, __func__,
               q->count, q->front, q->rear, start, *ptr);

        start++;
        start %= q->nmemb;
    }
    while (start != q->rear);

    return true;
}

static av_cold int bm_videoenc_close(AVCodecContext *avctx);


static void bmvpu_setup_logging(void)
{
    BmVpuEncLogLevel level = BMVPU_ENC_LOG_LEVEL_ERROR;

    int av_log_level = av_log_get_level();
    switch (av_log_level) {
    case AV_LOG_QUIET:
    case AV_LOG_PANIC:
    case AV_LOG_FATAL:
    case AV_LOG_ERROR:
        level = BMVPU_ENC_LOG_LEVEL_ERROR;
        break;
    case AV_LOG_WARNING:
        level = BMVPU_ENC_LOG_LEVEL_WARNING;
        break;
    case AV_LOG_INFO:
    case AV_LOG_VERBOSE:
        level = BMVPU_ENC_LOG_LEVEL_INFO;
        break;
    case AV_LOG_DEBUG:
        level = BMVPU_ENC_LOG_LEVEL_DEBUG;
        break;
    case AV_LOG_TRACE:
        level = BMVPU_ENC_LOG_LEVEL_TRACE;
        break;
    default:
        level = BMVPU_ENC_LOG_LEVEL_ERROR;
        break;
    }

    av_log(NULL, AV_LOG_DEBUG, "av_log_level: %d\n", av_log_level);
    av_log(NULL, AV_LOG_DEBUG, "bmvpuapi logging threshold: %d\n", level);

    bmvpu_enc_set_logging_threshold(level);
}

static void* acquire_output_buffer(void *context, size_t size, void **acquired_handle)
{
    bs_buffer_t* s = (bs_buffer_t*)(context);
    void *mem;
    int ret;
    ret = ff_alloc_packet(s->avctx, s->avpkt, size);
    if (ret < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Error! Failed ff_alloc_packet()!\n");
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

static BmVpuFramebuffer* get_src_framebuffer(AVCodecContext *avctx)
{
    int i;
    BmVpuEncContext* ctx = (BmVpuEncContext *)(avctx->priv_data);

    if (bm_queue_is_empty(ctx->frame_unused_queue))
    {
        av_log(NULL, AV_LOG_DEBUG, "frame_unused_queue is empty.\n");
        return NULL;
    }
    BmVpuFramebuffer *fb = *((BmVpuFramebuffer**)bm_queue_pop(ctx->frame_unused_queue));

    if (fb == NULL) {
        av_log(avctx, AV_LOG_ERROR, "frame buffer is NULL, pop\n");
        return NULL;
    }

    av_log(avctx, AV_LOG_TRACE, "myIndex = 0x%x, %p, pop\n", fb->myIndex, fb);
    for (i=0; i<ctx->num_src_fb; i++) {
        if (&(ctx->src_fb_list[i]) == fb)
            return fb;
    }

    return NULL;
}

static BmCustomMapOpt* get_roi_buffer(AVCodecContext *avctx)
{
    int i;
    BmVpuEncContext* ctx = (BmVpuEncContext *)(avctx->priv_data);

    if (bm_queue_is_empty(ctx->roi_unused_queue))
    {
        av_log(avctx, AV_LOG_DEBUG, "roi_unused_queue is empty.\n");
        return NULL;
    }
    BmCustomMapOpt *roi_map = *((BmCustomMapOpt**)bm_queue_pop(ctx->roi_unused_queue));

    if (roi_map == NULL) {
        av_log(avctx, AV_LOG_ERROR, "roi_map is NULL, pop\n");
        return NULL;
    }

    av_log(avctx, AV_LOG_TRACE, "myIndex = %p, pop\n", roi_map);
    for (i=0; i<ctx->num_roi; i++) {
        if (&(ctx->roi_list[i]) == roi_map)
            return roi_map;
    }

    return NULL;
}


#ifdef BM_PCIE_MODE
static int bm_image_upload(BmVpuEncoder* video_encoder,
                           uint64_t dst_data[4], int dst_linesizes[4],
                           const uint8_t *src_data[4], const int src_linesizes[4],
                           enum AVPixelFormat pix_fmt, int width, int height)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pix_fmt);
    int i, planes_nb = 0;
    int ret;

    for (i = 0; i < desc->nb_components; i++)
        planes_nb = FFMAX(planes_nb, desc->comp[i].plane + 1);

    for (i = 0; i < planes_nb; i++) {
        uint64_t  dst          = dst_data[i];
        ptrdiff_t dst_linesize = dst_linesizes[i];
        const uint8_t*  src    = src_data[i];
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

        av_log(NULL, AV_LOG_DEBUG,
               "plane %d: bwidth %zd, dst_linesize %zd, src_linesize %zd\n",
               i, bwidth, dst_linesize, src_linesize);

        ret = bmvpu_upload_data(video_encoder->soc_idx, src, src_linesize,
                                dst, dst_linesize, bwidth, h);
        // ret = bm_memcpy_s2d_partial(bmvpu_enc_get_bmlib_handle(video_encoder->soc_idx), dst, src, bwidth*h);
        if (ret != BM_SUCCESS) {
            av_log(NULL, AV_LOG_ERROR, "bm_memcpy_s2d_partial failed\n");
            return -1;
        }
    }

    return 0;
}
#endif

static int check_input_opt(AVCodecContext *avctx)
{
    int unit, mask;
    if (avctx->codec_id == AV_CODEC_ID_H264) {
        unit = 2;
    } else {
        unit = 8;
    }
    mask = unit - 1;
    if (avctx->width & mask) {
        av_log(avctx, AV_LOG_ERROR, "The encoding width(%d) is not multiple of %d!\n", avctx->width, unit);
        return AVERROR_PATCHWELCOME;
    }
    if (avctx->height & mask) {
        av_log(avctx, AV_LOG_ERROR, "The encoding height(%d) is not multiple of %d!\n", avctx->height, unit);
        return AVERROR_PATCHWELCOME;
    }
    if (avctx->width < 256) {
        av_log(avctx, AV_LOG_ERROR, "Invalid encoding width (%d < 256)!\n", avctx->width);
        return AVERROR_EXTERNAL;
    }
    if (avctx->height < 128) {
        av_log(avctx, AV_LOG_ERROR, "Invalid encoding height (%d < 128)!\n", avctx->height);
        return AVERROR_EXTERNAL;
    }

    return 0;
}

static av_cold int bm_videoenc_init(AVCodecContext *avctx)
{
    BmVpuEncContext* ctx = (BmVpuEncContext*)(avctx->priv_data);
    BmVpuEncOpenParams* eop = &(ctx->open_params);
    AVBmCodecDeviceContext *bmcodec_device_hwctx = NULL;
    int codec_fmt = 0;
    int raw_size = avctx->width * avctx->height * 3/2;
    int sw_pix_fmt;
    int ret, i;

    bmvpu_setup_logging();

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    ctx->hw_accel = 0;
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

    ctx->first_pkt_recevie_flag = 0;
#ifndef BM_PCIE_MODE
    ctx->soc_idx = 0;
#endif

    if (ctx->hw_accel)
        sw_pix_fmt = avctx->sw_pix_fmt;
    else
        sw_pix_fmt = avctx->pix_fmt;

    if ((sw_pix_fmt != AV_PIX_FMT_YUV420P) &&
        (sw_pix_fmt != AV_PIX_FMT_YUVJ420P) &&
        (sw_pix_fmt != AV_PIX_FMT_NV12) &&
        (sw_pix_fmt != AV_PIX_FMT_NV21)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%d", avctx->pix_fmt);
        av_log(avctx, AV_LOG_ERROR,
               "Specified encoder sw pixel format %s is not supported\n",
               (char *)av_x_if_null(av_get_pix_fmt_name(sw_pix_fmt), buf));
        return AVERROR_PATCHWELCOME;
    }

    if (ctx->hw_accel && avctx->pix_fmt == AV_PIX_FMT_BMCODEC) {
        ctx->zero_copy = 1;
    }

    ret = check_input_opt(avctx);
    if (ret != 0) {
        return ret;
    }

    /* Load the VPU firmware */
    ret = bmvpu_enc_load(ctx->soc_idx);
    if (ret != BM_VPU_ENC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "bmvpu_enc_load failed!\n");
        return AVERROR_EXTERNAL;
    }

    ctx->core_idx = bmvpu_enc_get_core_idx(ctx->soc_idx);
    ctx->is_end = false;

    /* Allocator for external DMA buffers.
     * If this is NULL, bmvpu_get_default_allocator() is used. */

#if 0
    /* The size for YUV 420/422/444 */
    if (avctx->pix_fmt == AV_PIX_FMT_YUV420P || avctx->pix_fmt == AV_PIX_FMT_NV12)
        raw_size = avctx->width * avctx->height * 3/2;
    else if (avctx->pix_fmt == AV_PIX_FMT_GRAY8)
        raw_size = avctx->width * avctx->height;
#endif

    ctx->avctx = avctx; // TODO

    /* setting encode param */
    av_log(avctx, AV_LOG_INFO,  "width        : %d\n", avctx->width);
    av_log(avctx, AV_LOG_INFO,  "height       : %d\n", avctx->height);
    av_log(avctx, AV_LOG_INFO,  "pix_fmt      : %s\n", av_get_pix_fmt_name(avctx->pix_fmt));
    if (ctx->hw_accel)
        av_log(avctx, AV_LOG_INFO,  "sw_pix_fmt   : %s\n", av_get_pix_fmt_name(avctx->sw_pix_fmt));
    av_log(avctx, AV_LOG_DEBUG, "gop size     : %d\n", avctx->gop_size);

    av_log(avctx, AV_LOG_INFO,  "sophon device: %d\n", ctx->soc_idx);
    av_log(avctx, AV_LOG_DEBUG, "preset       : %d\n", ctx->preset);
    av_log(avctx, AV_LOG_DEBUG, "gop preset   : %d\n", ctx->gop_preset);
    av_log(avctx, AV_LOG_DEBUG, "perf         : %d\n", ctx->perf);
    av_log(avctx, AV_LOG_DEBUG, "zero copy    : %s\n", ctx->zero_copy ? "Yes":"No");
    av_log(avctx, AV_LOG_DEBUG, "roi enable   : %d\n", ctx->roi_enable);
    av_log(avctx, AV_LOG_DEBUG, "vecmd queue  : %d\n", ctx->enc_cmd_queue);

    if (avctx->codec_id == AV_CODEC_ID_H264)
        codec_fmt = BM_VPU_CODEC_FORMAT_H264;
    else
        codec_fmt = BM_VPU_CODEC_FORMAT_H265;

    bmvpu_enc_set_default_open_params(eop, codec_fmt);

    eop->soc_idx = ctx->soc_idx;

    eop->frame_width  = avctx->width;
    eop->frame_height = avctx->height;
    eop->timebase_den = avctx->time_base.den;
    eop->timebase_num = avctx->time_base.num;
    av_log(avctx, AV_LOG_DEBUG,
           "timebase_den=%d, timebase_num=%d\n",
           eop->timebase_den, eop->timebase_num);
    eop->fps_num = avctx->time_base.den;
    eop->fps_den = avctx->time_base.num * avctx->ticks_per_frame;

    /* If this is 1, then Cb and Cr are interleaved in one shared chroma
     * plane, otherwise they are separated in their own planes.
     * See the BmVpuColorFormat documentation for the consequences of this. */
    if ((sw_pix_fmt == AV_PIX_FMT_YUV420P) ||
        (sw_pix_fmt == AV_PIX_FMT_YUVJ420P)) {
        eop->pix_format = BM_VPU_ENC_PIX_FORMAT_YUV420P;
    } else if (sw_pix_fmt == AV_PIX_FMT_NV12) {
        eop->pix_format = BM_VPU_ENC_PIX_FORMAT_NV12;
    } else if (sw_pix_fmt == AV_PIX_FMT_NV21) {
        eop->pix_format = BM_VPU_ENC_PIX_FORMAT_NV21;
    } else {
        av_log(avctx, AV_LOG_ERROR, "format(%d) is not support\n", sw_pix_fmt);
        return AVERROR_UNKNOWN;
    }

    if (ctx->cqp >= 0) {
        /* If this is set to 0, rate control is disabled, and
         * constant quality mode is active instead. */
        eop->bitrate = 0;
        eop->cqp = ctx->cqp;
    } else {
        /* bitrate in kbps. */
        eop->bitrate = avctx->bit_rate;

        /* Size of the vbv buffer, in bits. This is only used if rate control is active
         * (= the bitrate in BmVpuEncOpenParams is nonzero).
         * 0 means the buffer size constraints are not checked for.
         * Default value is 0. */
        eop->vbv_buffer_size = avctx->rc_buffer_size;
    }

    /* 0 : Custom mode, TODO
     * 1 : recommended encoder parameters (slow encoding speed, highest picture quality)
     * 2 : Boost mode (normal encoding speed, normal picture quality),
     * 3 : Fast mode (high encoding speed, low picture quality) */
    if (ctx->preset == 0)
        eop->enc_mode = 3;
    else if (ctx->preset == 1)
        eop->enc_mode = 2;
    else if (ctx->preset == 2)
        eop->enc_mode = 1;
    else
        eop->enc_mode = 2; // TODO

    if (avctx->gop_size >= 0)
        eop->intra_period = avctx->gop_size; // TODO
    eop->gop_preset = ctx->gop_preset;

    eop->roi_enable = ctx->roi_enable;
    eop->cmd_queue_depth = ctx->enc_cmd_queue;

    if (ctx->params_str) {
        AVDictionary    *dict = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, ctx->params_str, "=", ":", 0)) {
            while (1) {
                en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX);
                if (en==NULL)
                    break;
                ret = bmvpu_enc_param_parse(&ctx->open_params, en->key, en->value);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_WARNING,
                           "Error parsing option '%s = %s'.\n",
                            en->key, en->value);
                }
            }
            av_dict_free(&dict);
        }
    }
    /* register buffer_alloc_func and buffer_free_func for alloc buffer outside */
    eop->buffer_alloc_func = NULL;
    eop->buffer_free_func = NULL;
    eop->buffer_context = NULL;

    /* Retrieve information about the required bitstream buffer */
    bmvpu_enc_get_bitstream_buffer_info(&(ctx->bs_buffer_size), &(ctx->bs_buffer_alignment));

#define BS_MASK (1024*4-1)
    /*Recommended bs size, based on gop_preset*/
    unsigned int bs_buffer_arry[12] = {0, 7, 7, 7, 10, 13, 7, 7, 18, 7};
    ctx->bs_buffer_size = ((ctx->bs_buffer_size+BS_MASK)&(~BS_MASK)) * bs_buffer_arry[eop->gop_preset];

    ctx->bs_dma_buffer = (BmVpuEncDMABuffer *)av_mallocz(sizeof(BmVpuEncDMABuffer));
    ret = bmvpu_enc_dma_buffer_allocate(ctx->core_idx, ctx->bs_dma_buffer, ctx->bs_buffer_size);
    if (ret != BM_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "bmvpu_enc_dma_buffer_allocate failed! line=%d\n", __LINE__);
        return AVERROR_EXTERNAL;
    }

    /* Open an encoder instance, using the previously allocated bitstream buffer */
    ret = bmvpu_enc_open(&(ctx->video_encoder), eop, ctx->bs_dma_buffer, &(ctx->initial_info));
    if (ret != BM_VPU_ENC_RETURN_CODE_OK) {
        av_log(avctx, AV_LOG_ERROR, "bmvpu_enc_open failed!\n");
        ret = AVERROR_EXTERNAL;
        goto cleanup;
    }

    // /* Retrieve the initial information to allocate source and reconstruction
    //  * framebuffers for the encoding process. */
    // ret = bmvpu_enc_get_initial_info(ctx->video_encoder, &(ctx->initial_info));
    // if (ret != 0) {
    //     av_log(avctx, AV_LOG_ERROR, "bmvpu_enc_get_initial_info failed!\n");
    //     ret = AVERROR_EXTERNAL;
    //     goto cleanup;
    // }


    ctx->num_src_fb = ctx->initial_info.min_num_src_fb;
    avctx->delay = ctx->num_src_fb;
    if (ctx->zero_copy) {
#ifndef BM_PCIE_MODE
        av_log(avctx, AV_LOG_WARNING, "Minimum number of input frame buffers"
               " for BM video encoder: %u\n", ctx->num_src_fb);
#endif
    } else {
        av_log(avctx, AV_LOG_DEBUG, "num framebuffers for src: %u\n", ctx->num_src_fb);
    }

    ctx->src_fb_list = av_mallocz(sizeof(BmVpuFramebuffer) * ctx->num_src_fb);
    if (ctx->src_fb_list == NULL) {
        av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
        ret = AVERROR(ENOMEM);
        goto cleanup;
    }

    memset(&(ctx->enc_params), 0, sizeof(BmVpuEncParams));
    if (ctx->zero_copy) {
        for (i = 0; i < ctx->num_src_fb; ++i) {
            int src_id = i;
            bmvpu_fill_framebuffer_params(&(ctx->src_fb_list[i]),
                                          &(ctx->initial_info.src_fb),
                                          NULL, src_id, NULL);
        }
    } else {
        ctx->src_fb_dmabuffers = av_mallocz(sizeof(BmVpuEncDMABuffer*) * ctx->num_src_fb);
        if (ctx->src_fb_dmabuffers == NULL) {
            av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
            ret = AVERROR(ENOMEM);
            goto cleanup;
        }
        memset(ctx->src_fb_dmabuffers, 0, sizeof(BmVpuEncDMABuffer*) * ctx->num_src_fb);

        for (i = 0; i < ctx->num_src_fb; ++i) {
            int src_id = i;
            /* Allocate a DMA buffer for each raw input frame. */
            ctx->src_fb_dmabuffers[i] = (BmVpuEncDMABuffer *)av_mallocz(sizeof(BmVpuEncDMABuffer));
            ret = bmvpu_enc_dma_buffer_allocate(ctx->core_idx, ctx->src_fb_dmabuffers[i], ctx->initial_info.src_fb.size);
            if (ret != BM_SUCCESS) {
                av_free(ctx->src_fb_dmabuffers[i]);
                ctx->src_fb_dmabuffers[i] = NULL;
                av_log(avctx, AV_LOG_ERROR, "bmvpu_enc_dma_buffer_allocate failed! line=%d\n", __LINE__);
                return AVERROR_EXTERNAL;
            }
            bmvpu_dma_buffer_map(ctx->core_idx, ctx->src_fb_dmabuffers[i], BM_VPU_ENC_MAPPING_FLAG_READ|BM_VPU_ENC_MAPPING_FLAG_WRITE);
            memset(ctx->src_fb_dmabuffers[i]->virt_addr, 0, ctx->src_fb_dmabuffers[i]->size);
            bmvpu_dma_buffer_unmap(ctx->core_idx, ctx->src_fb_dmabuffers[i]);

            bmvpu_fill_framebuffer_params(&(ctx->src_fb_list[i]),
                                          &(ctx->initial_info.src_fb),
                                          ctx->src_fb_dmabuffers[i],
                                          src_id, NULL);
        }
    }

    // roi
    if (ctx->roi_enable == 1) {
        ctx->num_roi = ctx->initial_info.min_num_src_fb;
        ctx->roi_list = av_mallocz(sizeof(BmCustomMapOpt) * ctx->num_roi);
        if (ctx->roi_list == NULL) {
            av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
            ret = AVERROR(ENOMEM);
            goto cleanup;
        }

        ctx->roi_dmabuffers = av_mallocz(sizeof(BmVpuEncDMABuffer*) * ctx->num_roi);
        if (ctx->roi_dmabuffers == NULL) {
            av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
            ret = AVERROR(ENOMEM);
            goto cleanup;
        }

        unsigned int roi_size = (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) > (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM)?
                                (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) : (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM);
        for (i = 0; i < ctx->num_roi; ++i) {
            int src_id = i;
            /* Allocate a DMA buffer for each raw input frame. */
            ctx->roi_dmabuffers[i] = (BmVpuEncDMABuffer *)av_mallocz(sizeof(BmVpuEncDMABuffer));
            ret = bmvpu_enc_dma_buffer_allocate(ctx->core_idx, ctx->roi_dmabuffers[i], roi_size);
            if (ret != BM_SUCCESS) {
                av_free(ctx->roi_dmabuffers[i]);
                ctx->roi_dmabuffers[i] = NULL;
                av_log(avctx, AV_LOG_ERROR, "bmvpu_malloc_device_byte_heap failed! 675\n");
                return AVERROR_EXTERNAL;
            }

            ctx->roi_list[i].addrCustomMap = bmvpu_enc_dma_buffer_get_physical_address(ctx->roi_dmabuffers[i]);

        }
    } else {
        ctx->num_roi = 0;
        ctx->roi_list = NULL;
        ctx->roi_dmabuffers = NULL;
    }



    /* Create queue for source frame unused */
    ctx->frame_unused_queue = bm_queue_create(ctx->num_src_fb, sizeof(BmVpuFramebuffer*));
    if (ctx->frame_unused_queue == NULL) {
        av_log(avctx, AV_LOG_ERROR, "bm_queue_create failed\n");
        ret = AVERROR_UNKNOWN;
        goto cleanup;
    }
    for (i=0; i<ctx->num_src_fb; i++) {
        BmVpuFramebuffer *fb = &(ctx->src_fb_list[i]);
        bm_queue_push(ctx->frame_unused_queue, (void*)(&fb));
        av_log(avctx, AV_LOG_TRACE, "myIndex = 0x%x, %p, push\n", fb->myIndex, fb);
    }

    if (av_log_get_level() >= AV_LOG_TRACE)
        bm_queue_show(ctx->frame_unused_queue);

    /* Create avframe list for source frame */
    ctx->avframe_used_list = av_mallocz(ctx->num_src_fb*sizeof(AVFrame*));
    if (ctx->avframe_used_list == NULL) {
        av_log(avctx, AV_LOG_ERROR, "bm_queue_create failed\n");
        ret = AVERROR_UNKNOWN;
        goto cleanup;
    }

    /* Set up the encoding parameters */
    ctx->enc_params.acquire_output_buffer = acquire_output_buffer;
    ctx->enc_params.finish_output_buffer  = finish_output_buffer;
    ctx->enc_params.output_buffer_context = NULL;

    /* ---> Create queue for roi unused */
    if (ctx->roi_enable == 1) {
        ctx->roi_unused_queue = bm_queue_create(ctx->num_roi, sizeof(BmCustomMapOpt*));
        if (ctx->roi_unused_queue == NULL) {
            av_log(avctx, AV_LOG_ERROR, "bm_queue_create failed\n");
            ret = AVERROR_UNKNOWN;
            goto cleanup;
        }
        for (i=0; i<ctx->num_roi; i++) {
            BmCustomMapOpt *roi = &(ctx->roi_list[i]);
            bm_queue_push(ctx->roi_unused_queue, (void*)(&roi));
            av_log(avctx, AV_LOG_TRACE, "myIndex = 0x%x, %p, push\n", roi);
        }
        if (av_log_get_level() >= AV_LOG_TRACE)
            bm_queue_show(ctx->roi_unused_queue);
    } else {
        ctx->roi_unused_queue = NULL;
    }
    /* <--- Create queue for roi unused */


    memset(&(ctx->input_frame),  0, sizeof(ctx->input_frame));
    memset(&(ctx->output_frame), 0, sizeof(ctx->output_frame));
    memset(&(ctx->bs_buffer), 0, sizeof(bs_buffer_t));
    ctx->bs_buffer.avctx = avctx;

    if (ctx->perf) {
        ctx->total_time  = 0.0f;
        ctx->total_frame = 0L;
        ctx->ps.tv_sec   = 0;
        ctx->ps.tv_sec   = 0;
        ctx->pe.tv_sec   = 0;
        ctx->pe.tv_usec  = 0;
    }

    /* setup extra data information */
    if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) {
        avctx->extradata_size = ctx->video_encoder->headers_rbsp_size;
        if (avctx->extradata_size <= 0) {
            av_log(avctx, AV_LOG_ERROR, "Cannot encode headers.\n");
            ret = AVERROR_INVALIDDATA;
            goto cleanup;
        }

        avctx->extradata = av_mallocz(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!avctx->extradata) {
            av_log(avctx, AV_LOG_ERROR,
                   "Cannot allocate header of size %d.\n", avctx->extradata_size);
            ret = ENOMEM;
            goto cleanup;
        }

        memcpy(avctx->extradata, ctx->video_encoder->headers_rbsp, avctx->extradata_size);
    }

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return 0;

cleanup:
    bm_videoenc_close(avctx);

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);
    return ret;
}

static av_cold int bm_videoenc_close(AVCodecContext *avctx)
{
    BmVpuEncContext* ctx = (BmVpuEncContext *)(avctx->priv_data);
    int i, ret;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    if (ctx->perf) {
        gettimeofday(&ctx->pe, NULL);
        ctx->total_time = ((ctx->pe.tv_sec*1000.0 + ctx->pe.tv_usec/1000.0) -
                           (ctx->ps.tv_sec*1000.0 + ctx->ps.tv_usec/1000.0));
        if (ctx->total_time > 0.0f) {
            av_log(avctx, AV_LOG_INFO, "Frames encoded: %ld. Encoding speed: %.1ffps. Time used: %.3fsec.\n",
                   ctx->total_frame, ctx->total_frame*1000/ctx->total_time, ctx->total_time/1000.0f);
        }
    }

    if (ctx->video_encoder != NULL) {
        bmvpu_enc_close(ctx->video_encoder);
        ctx->video_encoder = NULL;
    }


    if (ctx->avframe_used_list != NULL) {
        av_free(ctx->avframe_used_list);
        ctx->avframe_used_list = NULL;
    }

    if (ctx->frame_unused_queue != NULL) {
        bm_queue_destroy(ctx->frame_unused_queue);
        ctx->frame_unused_queue = NULL;
    }

    if (ctx->roi_unused_queue != NULL) {
        bm_queue_destroy(ctx->roi_unused_queue);
        ctx->roi_unused_queue = NULL;
    }

    if (!ctx->zero_copy) {
        if (ctx->src_fb_dmabuffers) {
            for (i = 0; i < ctx->num_src_fb; ++i) {
                bmvpu_enc_dma_buffer_deallocate(ctx->core_idx, ctx->src_fb_dmabuffers[i]);
                av_free(ctx->src_fb_dmabuffers[i]);
            }
            av_free(ctx->src_fb_dmabuffers);
            ctx->src_fb_dmabuffers = NULL;
        }
    }

    for (i = 0; i < ctx->num_src_fb; ++i) {
    BmVpuFramebuffer* fb = &(ctx->src_fb_list[i]);
        if (ctx->zero_copy && fb->context != NULL) {
            AVFrame* input = (AVFrame*)(fb->context);
            if (input != NULL)
            {
                av_frame_free(&input);
                fb->context = NULL;
            }
        }
     }

    if (ctx->src_fb_list) {
        av_free(ctx->src_fb_list);
        ctx->src_fb_list = NULL;
    }

    if (ctx->bs_dma_buffer) {
        bmvpu_enc_dma_buffer_deallocate(ctx->core_idx, ctx->bs_dma_buffer);
        av_free(ctx->bs_dma_buffer);
        ctx->bs_dma_buffer = NULL;
    }

    /*--> release roi*/
    if (ctx->roi_dmabuffers) {
        for (i = 0; i < ctx->num_src_fb; ++i) {
            if(ctx->roi_dmabuffers[i] != NULL) {
                bmvpu_enc_dma_buffer_deallocate(ctx->core_idx, ctx->roi_dmabuffers[i]);
                av_free(ctx->roi_dmabuffers[i]);
                ctx->roi_dmabuffers[i] = NULL;
            }
        }
        av_free(ctx->roi_dmabuffers);
        ctx->roi_dmabuffers = NULL;
    }
    if (ctx->roi_list) {
        av_free(ctx->roi_list);
        ctx->roi_list = NULL;
    }
    /*<-- release roi*/



    /* Unload the VPU firmware */
    if (ctx->soc_idx >= 0)
    {
        ret = bmvpu_enc_unload(ctx->soc_idx);
        if (ret != BM_VPU_ENC_RETURN_CODE_OK) {
            av_log(avctx, AV_LOG_ERROR, "Failed to call bmvpu_enc_unload\n");
            return AVERROR_UNKNOWN;
        }else
            ctx->soc_idx = -1;
    }

    return 0;
}

static int SetMapData(AVCodecContext *avctx, BmCustomMapOpt **roi_map, int picWidth, int picHeight, AVBMRoiInfo *roi)
{
    int i, sumQp = 0;
    int h, w, ctuPos, initQp;
    BmVpuEncContext* ctx = (BmVpuEncContext *)(avctx->priv_data);
    *roi_map = get_roi_buffer(avctx);
    if (*roi_map == NULL) {
        return 0;
    }

    if (avctx->codec_id == AV_CODEC_ID_H264) {
        int mbAddr = 0;
        int MbWidth         = BM_VPU_ALIGN16(picWidth) >> 4;
        int MbHeight        = BM_VPU_ALIGN16(picHeight) >> 4;
        AvcEncCustomMap     customMapBuf[BM_MAX_MB_NUM];
        if (roi->customRoiMapEnable == 1) {
            int bufSize = MbWidth*MbHeight;

            for (h = 0; h < MbHeight; h++) {
                for (w = 0; w < MbWidth; w++) {
                    mbAddr = w + h*MbWidth;
                    customMapBuf[mbAddr].field.mb_qp = MAX(MIN(roi->field[mbAddr].H264.mb_qp & 0x3f, 51), 0);
                    sumQp += customMapBuf[mbAddr].field.mb_qp;
                }
            }
            (*roi_map)->roiAvgQp = (sumQp + (bufSize>>1)) / bufSize; // round off.
            (*roi_map)->customRoiMapEnable = 1;
        }
        if (roi->customModeMapEnable == 1) {
            for (h = 0; h < MbHeight; h++) {
                for (w = 0; w < MbWidth; w++) {
                    mbAddr = w + h*MbWidth;
                    customMapBuf[mbAddr].field.mb_force_mode = roi->field[mbAddr].H264.mb_force_mode & 0x3;
                }
            }
            (*roi_map)->customModeMapEnable = 1;
        }


        ctx->core_idx = bmvpu_enc_get_core_idx(ctx->soc_idx);

        unsigned int roi_size = (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) > (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM)?
                                (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) : (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM);
        BmVpuEncDMABuffer dma_buf;
        dma_buf.phys_addr = (*roi_map)->addrCustomMap;
        dma_buf.size      = roi_size;
        bmvpu_dma_buffer_map(ctx->core_idx, &dma_buf, BM_VPU_ENC_MAPPING_FLAG_READ|BM_VPU_ENC_MAPPING_FLAG_WRITE);
        memset(dma_buf.virt_addr, 0, roi_size);
        memcpy(dma_buf.virt_addr, customMapBuf, sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM);
        bmvpu_dma_buffer_unmap(ctx->core_idx, &dma_buf);


    } else if (avctx->codec_id == AV_CODEC_ID_H265) {
        EncCustomMap        customMapBuf[BM_MAX_CTU_NUM];  // custom map 1 CTU data = 64bits
        int ctuMapWidthCnt  = BM_VPU_ALIGN64(picWidth) >> 6;
        int ctuMapHeightCnt = BM_VPU_ALIGN64(picHeight) >> 6;

        if (roi->customRoiMapEnable == 1) {
            int bufSize = (BM_VPU_ALIGN64(picWidth) >> 5) * (BM_VPU_ALIGN64(picHeight) >> 5);
            for (h = 0; h < ctuMapHeightCnt; h++) {
                for (w = 0; w < ctuMapWidthCnt; w++) {
                    ctuPos = (h * ctuMapWidthCnt + w);
                    customMapBuf[ctuPos].field.sub_ctu_qp_0 = MAX(MIN(roi->field[ctuPos].HEVC.sub_ctu_qp_0 & 0x3f, 51), 0);
                    customMapBuf[ctuPos].field.sub_ctu_qp_1 = MAX(MIN(roi->field[ctuPos].HEVC.sub_ctu_qp_1 & 0x3f, 51), 0);
                    customMapBuf[ctuPos].field.sub_ctu_qp_2 = MAX(MIN(roi->field[ctuPos].HEVC.sub_ctu_qp_2 & 0x3f, 51), 0);
                    customMapBuf[ctuPos].field.sub_ctu_qp_3 = MAX(MIN(roi->field[ctuPos].HEVC.sub_ctu_qp_3 & 0x3f, 51), 0);
                    sumQp += (customMapBuf[ctuPos].field.sub_ctu_qp_0 + customMapBuf[ctuPos].field.sub_ctu_qp_1 + customMapBuf[ctuPos].field.sub_ctu_qp_2 + customMapBuf[ctuPos].field.sub_ctu_qp_3);
                }
            }
            (*roi_map)->roiAvgQp = (sumQp + (bufSize>>1)) / bufSize; // round off.
            (*roi_map)->customRoiMapEnable = 1;
        }
        if (roi->customLambdaMapEnable == 1) {
            for (h = 0; h < ctuMapHeightCnt; h++) {
                for (w = 0; w < ctuMapWidthCnt; w++) {
                    ctuPos = (h * ctuMapWidthCnt + w);
                    customMapBuf[ctuPos].field.lambda_sad_0 = roi->field[ctuPos].HEVC.lambda_sad_0 & 0xff;
                    customMapBuf[ctuPos].field.lambda_sad_1 = roi->field[ctuPos].HEVC.lambda_sad_1 & 0xff;
                    customMapBuf[ctuPos].field.lambda_sad_2 = roi->field[ctuPos].HEVC.lambda_sad_2 & 0xff;
                    customMapBuf[ctuPos].field.lambda_sad_3 = roi->field[ctuPos].HEVC.lambda_sad_3 & 0xff;
                }
            }
            (*roi_map)->customLambdaMapEnable = 1;
        }

        if (roi->customModeMapEnable == 1) {
            for (h = 0; h < ctuMapHeightCnt; h++) {
                for (w = 0; w < ctuMapWidthCnt; w++) {
                    ctuPos = (h * ctuMapWidthCnt + w);
                    customMapBuf[ctuPos].field.ctu_force_mode = roi->field[ctuPos].HEVC.ctu_force_mode & 0x3;
                }
            }
            (*roi_map)->customModeMapEnable = 1;
        }

        if (roi->customCoefDropEnable == 1) {
            for (h = 0; h < ctuMapHeightCnt; h++) {
                for (w = 0; w < ctuMapWidthCnt; w++) {
                    ctuPos = (h * ctuMapWidthCnt + w);
                    customMapBuf[ctuPos].field.ctu_coeff_drop = roi->field[ctuPos].HEVC.ctu_coeff_drop & 0x1;
                }
            }
            (*roi_map)->customCoefDropEnable = 1;
        }

        ctx->core_idx = bmvpu_enc_get_core_idx(ctx->soc_idx);

        unsigned int roi_size = (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) > (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM)?
                                (sizeof(EncCustomMap)*BM_MAX_CTU_NUM) : (sizeof(AvcEncCustomMap)*BM_MAX_MB_NUM);
        BmVpuEncDMABuffer dma_buf;
        dma_buf.phys_addr = (*roi_map)->addrCustomMap;
        dma_buf.size      = roi_size;
        bmvpu_dma_buffer_map(ctx->core_idx, &dma_buf, BM_VPU_ENC_MAPPING_FLAG_READ|BM_VPU_ENC_MAPPING_FLAG_WRITE);
        memset(dma_buf.virt_addr, 0, roi_size);
        memcpy(dma_buf.virt_addr, customMapBuf, sizeof(EncCustomMap)*BM_MAX_CTU_NUM);
        bmvpu_dma_buffer_unmap(ctx->core_idx, &dma_buf);

    }

    return 0;
}

static int bm_videoenc_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                                    const AVFrame *frame, int *got_packet)
{
    BmVpuEncContext* ctx = (BmVpuEncContext *)(avctx->priv_data);
    AVFrame* pic = NULL;
    // bm_device_mem_t wrapped_dmem;
    BmVpuEncDMABuffer wrapped_dmem;
    BmVpuEncReturnCodes enc_ret = -1;
    unsigned int output_code = 0; // TODO
    int i, ret = 0;
    int pict_type;
    int send_frame_status = 0;
    int get_stream_times  = 0;

    av_log(avctx, AV_LOG_TRACE, "Enter %s\n", __func__);

    if (ctx->perf && ctx->total_frame == 0) {
        gettimeofday(&ctx->ps, NULL);
    }

    *got_packet = 0;
    if (frame) {
        const int width  = frame->width;
        const int height = frame->height;
        int sw_format;
        char buf[128];

        if (frame->format != avctx->pix_fmt) {
            av_log(avctx, AV_LOG_ERROR,
                   "The frame pixel format(%s) is different from that(%s) of encoder context.\n",
                   av_get_pix_fmt_name(frame->format),
                   av_get_pix_fmt_name(avctx->pix_fmt));
            return AVERROR_EXTERNAL;
        }
        if (ctx->hw_accel) {
            AVHWFramesContext* frames_ctx = NULL;
            if (frame->hw_frames_ctx==NULL || frame->hw_frames_ctx->data==NULL) {
                av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid frame hw_frames_ctx(NULL)!\n");
                return AVERROR_EXTERNAL;
            }

            frames_ctx = (AVHWFramesContext*)frame->hw_frames_ctx->data;
            if (frame->format != AV_PIX_FMT_BMCODEC) {
                av_log(avctx, AV_LOG_ERROR,
                       "The frame pixel hw format(%s) is not \'%s\'\n",
                       av_get_pix_fmt_name(frame->format),
                       av_get_pix_fmt_name(AV_PIX_FMT_BMCODEC));
                return AVERROR_EXTERNAL;
            }
            sw_format = frames_ctx->sw_format;
            if (sw_format != avctx->sw_pix_fmt) {
                av_log(avctx, AV_LOG_ERROR,
                       "The frame pixel sw format(%s) is different from that(%s) of encoder context.\n",
                       av_get_pix_fmt_name(sw_format),
                       av_get_pix_fmt_name(avctx->sw_pix_fmt));
                return AVERROR_EXTERNAL;
            }
        } else {
            sw_format = frame->format;
        }

        if (sw_format != AV_PIX_FMT_YUV420P && sw_format != AV_PIX_FMT_YUVJ420P && sw_format != AV_PIX_FMT_NV12 && sw_format != AV_PIX_FMT_NV21) {
            snprintf(buf, sizeof(buf), "%d", sw_format);
            av_log(avctx, AV_LOG_ERROR,
                   "B. Specified pixel format %s is not supported\n",
                   (char *)av_x_if_null(av_get_pix_fmt_name(sw_format), buf));
            return AVERROR_PATCHWELCOME;
        }

        /* Get data from upstream */
        pic = av_frame_clone(frame);
        if (!pic)
            return AVERROR(ENOMEM);

        ctx->src_fb = get_src_framebuffer(avctx);
        if (ctx->src_fb == NULL) {
            av_log(avctx, AV_LOG_ERROR, "get_src_framebuffer failed\n");
            av_packet_unref(avpkt);
            return AVERROR_UNKNOWN;
        }

        if (ctx->first_pkt_recevie_flag != 1) {
            if (bm_queue_is_empty(ctx->frame_unused_queue)) {
                ctx->first_pkt_recevie_flag = 1;
            }
        }

        av_log(avctx, AV_LOG_DEBUG, "width =%d\n", width);
        av_log(avctx, AV_LOG_DEBUG, "height=%d\n", height);

        for (i=0; i<8; i++)
            av_log(avctx, AV_LOG_TRACE, "data %d: %p, line size %d: %d\n",
                   i, pic->data[i], i, pic->linesize[i]);

        if (ctx->hw_accel) {
            AVBmCodecFrame* hwpic = (AVBmCodecFrame*)pic->data[4];
            int y_size, c_size, total_size;

            if (hwpic && hwpic->maptype == 1) {
                av_log(avctx, AV_LOG_ERROR,
                       "For now, video encoder does NOT support compressed frame data!\n"
                       "Please use scale_bm to decompresse the data to uncompressed data first!\n");
                return AVERROR_EXTERNAL;
            }

            av_log(avctx, AV_LOG_TRACE, "The input pixel format is %s(%s)!\n",
                   av_get_pix_fmt_name(pic->format), av_get_pix_fmt_name(sw_format));

            av_log(avctx, AV_LOG_TRACE, "data 0: %p\n", pic->data[0]);
            av_log(avctx, AV_LOG_TRACE, "data 1: %p\n", pic->data[1]);
            av_log(avctx, AV_LOG_TRACE, "data 2: %p\n", pic->data[2]);

            av_log(avctx, AV_LOG_TRACE, "line size 0: %d\n", pic->linesize[0]);
            av_log(avctx, AV_LOG_TRACE, "line size 1: %d\n", pic->linesize[1]);
            av_log(avctx, AV_LOG_TRACE, "line size 2: %d\n", pic->linesize[2]);


            for (i=0; i<(((sw_format==AV_PIX_FMT_YUV420P)||(sw_format==AV_PIX_FMT_YUVJ420P))?3:2); i++) {
                if (pic->data[i] == NULL) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic data[%d](%p)!\n", i, pic->data[i]);
                    return AVERROR_EXTERNAL;
                }
                if (pic->linesize[i] <= 0) {
                    av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic linesize[%d](%d]!\n", i, pic->linesize[i]);
                    return AVERROR_EXTERNAL;
                }
            }

            y_size = pic->data[1] - pic->data[0];
            if (sw_format == AV_PIX_FMT_NV12 || sw_format == AV_PIX_FMT_NV21) {
                c_size = y_size/2; // TODO
                total_size = y_size + c_size;
            } else {
                c_size = pic->data[2] - pic->data[1];
                total_size = y_size + c_size*2;
            }

            // /* The EXTERNAL DMA buffer filled with frame data */
            wrapped_dmem.phys_addr = (unsigned long)(pic->data[0]);
            // /* The size of EXTERNAL DMA buffer */
            wrapped_dmem.size = total_size;
            wrapped_dmem.enable_cache = 0;

            ctx->src_fb->y_stride    = pic->linesize[0];
            ctx->src_fb->cbcr_stride = pic->linesize[1];

            ctx->src_fb->width       = pic->width;
            ctx->src_fb->height      = pic->height;

            ctx->src_fb->y_offset    = 0;
            ctx->src_fb->cb_offset   = y_size;
            ctx->src_fb->cr_offset   = y_size + c_size;

            ctx->src_fb->dma_buffer  = (BmVpuEncDMABuffer *)&wrapped_dmem;

        } else if (!ctx->zero_copy) {
            /* The input frames come in Non-DMA memory */
            uint8_t* addr;
            uint8_t *src_y, *src_u, *src_v;
            uint8_t *dst_y, *dst_u, *dst_v;
            int src_stride_y, src_stride_u, src_stride_v;
            int dst_stride_y, dst_stride_u, dst_stride_v;

            if (pic->data[0] == NULL || pic->data[1] == NULL ||
                (((pic->format == AV_PIX_FMT_YUV420P)||(pic->format == AV_PIX_FMT_YUVJ420P)) && pic->data[2] == NULL)) {
                av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic data!\n");
                return AVERROR(EINVAL);;
            }

            if (pic->linesize[0] <= 0 || pic->linesize[1] <= 0 ||
               ((pic->format == AV_PIX_FMT_YUV420P ||(pic->format == AV_PIX_FMT_YUVJ420P)) && pic->linesize[2] <= 0)) {
                av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic line size!\n");
                return AVERROR(EINVAL);;
            }

            /* Load the input pixels into the DMA buffer */
#ifdef BM_PCIE_MODE
            addr = (uint8_t*)bmvpu_enc_dma_buffer_get_physical_address(ctx->src_fb->dma_buffer);
            if (addr == NULL) {
                av_log(avctx, AV_LOG_ERROR, "bm_mem_get_device_addr failed\n");
                return AVERROR_EXTERNAL;
            }
#else
            ret = bmvpu_dma_buffer_map(ctx->core_idx, ctx->src_fb->dma_buffer, BM_VPU_ENC_MAPPING_FLAG_READ|BM_VPU_ENC_MAPPING_FLAG_WRITE);
            if (ret != BM_SUCCESS) {
                av_log(avctx, AV_LOG_ERROR, "bmvpu_dma_buffer_map failed\n");
                return AVERROR_EXTERNAL;
            }
            addr = (uint8_t *)(ctx->src_fb->dma_buffer->virt_addr);
#endif
            dst_y = addr + ctx->src_fb-> y_offset;
            dst_u = addr + ctx->src_fb->cb_offset;
            dst_v = addr + ctx->src_fb->cr_offset;
            dst_stride_y = ctx->src_fb->   y_stride;
            dst_stride_u = ctx->src_fb->cbcr_stride;
            dst_stride_v = ctx->src_fb->cbcr_stride;

            /* Copy YUV420P to YUV420P */
            src_y = pic->data[0];
            src_u = pic->data[1];
            src_v = pic->data[2];
            src_stride_y = pic->linesize[0];
            src_stride_u = pic->linesize[1];
            src_stride_v = pic->linesize[2];

            av_log(avctx, AV_LOG_TRACE, "%s: copying...\n", __func__);
            {
                const uint8_t* src_data[4] = {src_y, src_u, src_v, NULL};
                const int src_linesizes[4] = {src_stride_y, src_stride_u, src_stride_v, 0};
                int dst_linesizes[4] = {dst_stride_y, dst_stride_u, dst_stride_v, 0};

#ifdef BM_PCIE_MODE
                uint64_t dst_data[4] = {(uint64_t)dst_y, (uint64_t)dst_u, (uint64_t)dst_v, 0L};

                ret = bm_image_upload(ctx->video_encoder,
                                      dst_data, dst_linesizes,
                                      src_data, src_linesizes,
                                      pic->format,
                                      width, height);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "bm_image_upload failed\n");
                    return AVERROR_EXTERNAL;
                }
#else
                uint8_t* dst_data[4] = {dst_y, dst_u, dst_v, NULL};

                av_image_copy(dst_data, dst_linesizes,
                              src_data, src_linesizes,
                              pic->format,
                              width, height);
#endif
            }
            av_log(avctx, AV_LOG_TRACE, "%s: copying...Done\n", __func__);

#ifndef BM_PCIE_MODE
            // /* Flush cache to DMA buffer */
            // bmvpu_enc_dma_buffer_flush(ctx->core_idx, ctx->src_fb->dma_buffer);

            /* Unmap the DMA buffer of the raw frame */
            bmvpu_dma_buffer_unmap(ctx->core_idx, ctx->src_fb->dma_buffer);

#endif
        } else { /* The input frames come from DMA memory defined by user */
            int y_size, c_size, total_size;

            if (pic->data[4] == NULL || pic->data[5] == NULL ||
               ((pic->format == AV_PIX_FMT_YUV420P ||(pic->format == AV_PIX_FMT_YUVJ420P)) && pic->data[6] == NULL)) {
                av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic data!\n");
                return AVERROR_EXTERNAL;
            }

            if (pic->linesize[4] <= 0 || pic->linesize[5] <= 0 ||
               ((pic->format == AV_PIX_FMT_YUV420P||(pic->format == AV_PIX_FMT_YUVJ420P)) && pic->linesize[6] <= 0)) {
                av_log(avctx, AV_LOG_ERROR, "ERROR! Invalid pic line size!\n");
                return AVERROR_EXTERNAL;
            }

            if ((pic->format == AV_PIX_FMT_YUV420P) ||
                (pic->format == AV_PIX_FMT_YUVJ420P)) {
                y_size = pic->data[5] - pic->data[4];
                c_size = pic->data[6] - pic->data[5];
                total_size = y_size + c_size*2;
            } else {
                y_size = pic->data[5] - pic->data[4];
                c_size = y_size/2; // TODO
                total_size = y_size + c_size;
            }

            // wrapped_dmem.u.device.dmabuf_fd = 1;
            // /* The EXTERNAL DMA buffer filled with frame data */
            // wrapped_dmem.u.device.device_addr = (unsigned long)(pic->data[4]);
            // /* The size of EXTERNAL DMA buffer */
            // wrapped_dmem.size = total_size;

            wrapped_dmem.phys_addr = (unsigned long)(pic->data[4]);
            // wrapped_dmem.virt_addr = ;
            wrapped_dmem.size = total_size;
            // wrapped_dmem.enable_cache = 0;

            ctx->src_fb->y_stride    = pic->linesize[4];
            ctx->src_fb->cbcr_stride = pic->linesize[5];

            ctx->src_fb->width       = pic->width;
            ctx->src_fb->height      = pic->height;

            ctx->src_fb->y_offset    = 0;
            ctx->src_fb->cb_offset   = y_size;
            ctx->src_fb->cr_offset   = y_size + c_size;
            ctx->src_fb->dma_buffer  = (BmVpuEncDMABuffer *)&wrapped_dmem;
        }

        ctx->enc_params.skip_frame = 0; // TODO

        ctx->input_frame.framebuffer = ctx->src_fb;
        if (ctx->zero_copy)
            ctx->src_fb->context     = (void*)pic;
        else
            ctx->input_frame.context = NULL;
        ctx->input_frame.pts = pic->pts;
        ctx->input_frame.dts = pic->pkt_dts; // TODO

        av_log(avctx, AV_LOG_DEBUG, "input frame: context=%p, pts=%ld, dts=%ld\n",
               ctx->input_frame.context,
               ctx->input_frame.pts,
               ctx->input_frame.dts);

        /* For non-dma-buffer case, the data has been copyed to the internal
         * dma buffer of encoder. So unref and free the input frame now. */
        if (!ctx->zero_copy)
            av_frame_free(&pic);
    } else {
        if (ctx->is_end)
            return  0;

        ctx->input_frame.framebuffer = NULL;
        ctx->input_frame.context = NULL;
        ctx->input_frame.pts = 0L;
        ctx->input_frame.dts = 0L;
    }

    ctx->bs_buffer.avpkt = avpkt;
    ctx->enc_params.output_buffer_context = (void*)(&(ctx->bs_buffer));

    if (frame != NULL) {
        AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_BM_ROI_INFO);
        if (sd) {
            AVBMRoiInfo *roi = (AVBMRoiInfo*)sd->data;
            if (SetMapData(avctx, &(ctx->enc_params.customMapOpt), frame->width, frame->height, roi) != 0) {
                return AVERROR_INVALIDDATA;
            }
        }
    }
re_send:
    if (ctx->input_frame.framebuffer == NULL) {
        send_frame_status = bmvpu_enc_send_frame(ctx->video_encoder, &(ctx->input_frame), &(ctx->enc_params));
    } else {
        send_frame_status = bmvpu_enc_send_frame(ctx->video_encoder, &(ctx->input_frame), &(ctx->enc_params));
    }
    if (enc_ret == 0) {
        usleep(1000);
        goto enc_end;
    }

get_stream:
    enc_ret = bmvpu_enc_get_stream(ctx->video_encoder, &(ctx->output_frame), &(ctx->enc_params));
    if (enc_ret == BM_VPU_ENC_RETURN_CODE_ENC_END) {
        av_log(avctx, AV_LOG_DEBUG, "encoding end!\n");
        ctx->is_end = true;
        return 0;
    }
    if ((ctx->output_frame.data_size > 0) && (enc_ret >= 0)) {
        // 1. Collect the input frames released
        for (i=0; i<ctx->num_src_fb; i++) {
            BmVpuFramebuffer* fb = &(ctx->src_fb_list[i]);
            if (ctx->output_frame.src_idx != fb->myIndex) {
                continue;
            }
            if (ctx->zero_copy) {
                AVFrame* input = (AVFrame*)(fb->context);
                if (input == NULL) {
                    av_log(avctx, AV_LOG_ERROR, "context is NULL in output frame\n");
                    return AVERROR(EFAULT);
                }
                av_log(avctx, AV_LOG_TRACE, "context is %p in output frame\n",
                       ctx->output_frame.context);

                /* unref and release the input frame referenced */
                av_frame_free(&input); // TODO
                fb->context = NULL;
            }
            bm_queue_push(ctx->frame_unused_queue, &fb);
            av_log(avctx, AV_LOG_TRACE, "myIndex = 0x%x, push\n", fb->myIndex);
            break;
        }

        // 2. release roi phyaddr
        for (int j=0; j<ctx->num_roi; j++) {
            BmCustomMapOpt* roi = &(ctx->roi_list[j]);
            if (roi->addrCustomMap != ctx->output_frame.u64CustomMapPhyAddr)
                continue;

            bm_queue_push(ctx->roi_unused_queue, &roi);
            break;
        }

        // // 3. save pkt data
        // ret = ff_alloc_packet(avctx, avpkt, ctx->output_frame.data_size);
        // if (ret < 0) {
        //     av_log(avctx, AV_LOG_ERROR, "Error! Failed ff_alloc_packet()!\n");
        //     return NULL;
        // }
        // memcpy(avpkt->data, ctx->output_frame.data, ctx->output_frame.data_size);
        av_log(avctx, AV_LOG_DEBUG, "output frame: context=%p, pts=%ld, dts=%ld\n",
               ctx->output_frame.context,
               ctx->output_frame.pts,
               ctx->output_frame.dts);

        if (ctx->output_frame.frame_type == BM_VPU_ENC_FRAME_TYPE_IDR ||
            ctx->output_frame.frame_type == BM_VPU_ENC_FRAME_TYPE_I)
            avpkt->flags |= AV_PKT_FLAG_KEY;

        avpkt->pts = ctx->output_frame.pts;
        avpkt->dts = ctx->output_frame.dts; // TODO
        switch (ctx->output_frame.frame_type) {
        case BM_VPU_ENC_FRAME_TYPE_IDR:
        case BM_VPU_ENC_FRAME_TYPE_I:
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_VPU_ENC_FRAME_TYPE_P:
            pict_type = AV_PICTURE_TYPE_P;
            break;
        case BM_VPU_ENC_FRAME_TYPE_B:
            pict_type = AV_PICTURE_TYPE_B;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
        }

        ff_side_data_set_encoder_stats(avpkt, ctx->output_frame.avg_ctu_qp * FF_QP2LAMBDA, NULL, 0, pict_type);

        /* output the encoded frame. */
        *got_packet = 1;

        if (ctx->perf)
            ctx->total_frame++;
    } else {
        av_packet_unref(avpkt);
        if (ctx->first_pkt_recevie_flag == 1) {
            if (get_stream_times < GET_STREAM_TIMEOUT) {
                usleep(1000*5);
                get_stream_times++;
                goto get_stream;
            }
        }
    }

enc_end:
    if ((ctx->input_frame.framebuffer != NULL) && (send_frame_status != BM_VPU_ENC_RETURN_CODE_OK)) {
        usleep(10);
        goto re_send;
    }

    // the last frame need wait 3s.
    if ((ctx->input_frame.framebuffer == NULL) && (*got_packet == 0) && (ctx->is_end == 0)) {
        if (get_stream_times < GET_STREAM_TIMEOUT) {
            av_packet_unref(avpkt);
            usleep(100);
            get_stream_times++;
            goto get_stream;
        }
    }

    av_log(avctx, AV_LOG_TRACE, "Leave %s\n", __func__);

    return ret;
}

#define OFFSET(x) offsetof(BmVpuEncContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] = {
    { "is_dma_buffer", "flag to indicate if the input frame buffer is continuous physical buffer",
        OFFSET(zero_copy), AV_OPT_TYPE_FLAGS, {.i64 = 1}, 0, 1, FLAGS },
    { "preset", "Set the encoding preset : 0 - fast, 1 - medium, 2 - slow(default). (deprecated, please set it in enc-params)",
        OFFSET(preset), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 2, FLAGS },
    { "gop_preset", "gop preset idx: (deprecated, please set it in enc-params)\n "
        "\t\t\t\t\t\t1 - all I, gopsize 1;\n"
        "\t\t\t\t\t\t2 - IPP, cyclic gopsize 1;\n"
        "\t\t\t\t\t\t3 - IBB, cyclic gopsize 1;\n"
        "\t\t\t\t\t\t4 - IBPBP, cyclic gopsize 2;\n"
        "\t\t\t\t\t\t5 - IBBBP, cyclic gopsize 4;\n"
        "\t\t\t\t\t\t6 - IPPPP, cyclic gopsize 4;\n"
        "\t\t\t\t\t\t7 - IBBBB, cyclic gopsize 4;\n"
        "\t\t\t\t\t\t8 - random access, IBBBBBBBB, cyclic gopsize 8;\n"
        "\t\t\t\t\t\t9 - IPP,   cyclic gopsize 1;",
        OFFSET(gop_preset), AV_OPT_TYPE_INT, {.i64=2}, 1, 9, FLAGS },
#ifdef BM_PCIE_MODE
    { "sophon_idx", "Sophon device index when running in pcie mode.",
        OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, INT_MIN, INT_MAX, FLAGS },
#endif
    { "qp", "Constant quantization parameter rate control method",
        OFFSET(cqp), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 51, FLAGS }, // TODO
    { "enc-params",  "Override the configuration using a :-separated list of key=value parameters",
        OFFSET(params_str), AV_OPT_TYPE_STRING, { 0 }, 0, 0, FLAGS },
    { "perf", "flag to indicate if do the performance testing",
        OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "roi_enable", "roi enable: 0 disable roi encoding; 1 use roi encoding.  default 0.",
        OFFSET(roi_enable), AV_OPT_TYPE_INT, {.i64=0}, 0, 1, FLAGS },
    { "enc_cmd_queue", "vpu enc command queue depth",
        OFFSET(enc_cmd_queue), AV_OPT_TYPE_INT, { .i64 = 4 }, 1, 16, FLAGS }, // TODO
    { NULL},
};

#if CONFIG_H264_BM_ENCODER
static const AVClass bm_h264enc_class = {
    .class_name = "h264_bm_encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if defined(BM1684) || defined(BM1686)
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
#else
#define bmcodec_hw_configs NULL
#endif

const FFCodec ff_h264_bm_encoder = {
    .p.name           = "h264_bm",
    CODEC_LONG_NAME("BM H.264 encoder"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_H264,
    .priv_data_size = sizeof(BmVpuEncContext),
    .init           = bm_videoenc_init,
    .close          = bm_videoenc_close,
    FF_CODEC_ENCODE_CB(bm_videoenc_encode_frame),
    .p.priv_class   = &bm_h264enc_class,
    .p.capabilities = AV_CODEC_CAP_HARDWARE|AV_CODEC_CAP_DELAY,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .hw_configs     = bmcodec_hw_configs,
    .p.pix_fmts      = (const enum AVPixelFormat[]){
        AV_PIX_FMT_BMCODEC,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_NV21,
        AV_PIX_FMT_NONE
    }
};
#endif

#if CONFIG_H265_BM_ENCODER
static const AVClass bm_h265enc_class = {
    .class_name = "h265_bm_encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_h265_bm_encoder = {
    .p.name         = "h265_bm",
    CODEC_LONG_NAME("BM H.265 encoder"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_H265,
    .priv_data_size = sizeof(BmVpuEncContext),
    .init           = bm_videoenc_init,
    .close          = bm_videoenc_close,
    .hw_configs     = bmcodec_hw_configs,
    FF_CODEC_ENCODE_CB(bm_videoenc_encode_frame),
    .p.priv_class   = &bm_h265enc_class,
    .p.capabilities = AV_CODEC_CAP_HARDWARE|AV_CODEC_CAP_DELAY,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .p.pix_fmts     = (const enum AVPixelFormat[]){
        AV_PIX_FMT_BMCODEC,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_NV21,
        AV_PIX_FMT_NONE
    },
    .p.wrapper_name   = "bm"
};

static const AVClass bm_hevcenc_class = {
    .class_name = "hevc_bm_encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_hevc_bm_encoder = {
    .p.name           = "hevc_bm",
    CODEC_LONG_NAME("BM HEVC encoder"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_H265,
    .priv_data_size = sizeof(BmVpuEncContext),
    .init           = bm_videoenc_init,
    .close          = bm_videoenc_close,
    .hw_configs     =bmcodec_hw_configs,
    FF_CODEC_ENCODE_CB(bm_videoenc_encode_frame),
    .p.priv_class     = &bm_hevcenc_class,
    .p.capabilities   = AV_CODEC_CAP_HARDWARE|AV_CODEC_CAP_DELAY,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .p.pix_fmts       = (const enum AVPixelFormat[]){
        AV_PIX_FMT_BMCODEC,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_NV21,
        AV_PIX_FMT_NONE
    },
    .p.wrapper_name   = "bm"
};
#endif

