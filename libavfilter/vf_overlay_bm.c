#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_bmcodec.h"
#include "libavutil/opt.h"
#include "libavutil/eval.h"
#include "libavutil/pixdesc.h"

#include "avfilter.h"
#include "filters.h"
#include "framesync.h"
#include "bmcv_api_ext_c.h"
#include "bmlib_runtime.h"


#define MAIN    0
#define OVERLAY 1

#define USEING_MEM_HEAP2 4
#define USEING_MEM_HEAP1 2

enum var_name {
    VAR_MAIN_W,    VAR_MW,
    VAR_MAIN_H,    VAR_MH,
    VAR_OVERLAY_W, VAR_OW,
    VAR_OVERLAY_H, VAR_OH,
    VAR_X,
    VAR_Y,
    VAR_N,
    VAR_POS,
    VAR_T,
    VAR_VARS_NB
};

enum EvalMode {
    EVAL_MODE_INIT,
    EVAL_MODE_FRAME,
    EVAL_MODE_NB
};

static const char *const var_names[] = {
    "main_w",    "W", ///< width  of the main    video
    "main_h",    "H", ///< height of the main    video
    "overlay_w", "w", ///< width  of the overlay video
    "overlay_h", "h", ///< height of the overlay video
    "x",
    "y",
    "n",            ///< number of frame
    "pos",          ///< position in the file
    "t",            ///< timestamp expressed in seconds
    NULL
};

typedef struct buffer_t {
    bm_image* img;
    uint8_t* host_mem;
} buffer_t;

typedef struct OverlayBmContext {
    const AVClass *class;

    int sophon_idx;
    bm_handle_t handle;

    int zero_copy;

    FFFrameSync fs;
    AVBufferRef *hw_device_ctx;

    int eval_mode;
    int x_position;
    int y_position;

    double var_values[VAR_VARS_NB];
    char *x_expr, *y_expr;
    AVExpr *x_pexpr, *y_pexpr;
} OverlayBmContext;

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

static inline int normalize_xy(double d, int chroma_sub)
{
    if (isnan(d))
        return INT_MAX;
    return (int)d & ~((1 << chroma_sub) - 1);
}

static void eval_expr(AVFilterContext *ctx)
{
    OverlayBmContext *s = ctx->priv;

    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->var_values[VAR_Y] = av_expr_eval(s->y_pexpr, s->var_values, NULL);
    /* necessary if x is expressed from y  */
    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);

    s->x_position = normalize_xy(s->var_values[VAR_X], 1);

    s->y_position = s->var_values[VAR_Y];
}

static int set_expr(AVExpr **pexpr, const char *expr, const char *option, void *log_ctx)
{
    int ret;
    AVExpr *old = NULL;

    if (*pexpr)
        old = *pexpr;
    ret = av_expr_parse(pexpr, expr, var_names,
                        NULL, NULL, NULL, NULL, 0, log_ctx);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR,
               "Error when evaluating the expression '%s' for %s\n",
               expr, option);
        *pexpr = old;
        return ret;
    }

    av_expr_free(old);
    return 0;
}

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

static int map_avformat_to_bmformat(int avformat)
{
    int format;
    switch(avformat){
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUVJ420P:
            format = FORMAT_YUV420P; break;
        case AV_PIX_FMT_YUV422P:
        case AV_PIX_FMT_YUVJ422P:
            format = FORMAT_YUV422P; break;
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_YUVJ444P:
            format = FORMAT_YUV444P; break;
        case AV_PIX_FMT_NV12:    format = FORMAT_NV12; break;
        case AV_PIX_FMT_NV16:    format = FORMAT_NV16; break;
        case AV_PIX_FMT_GRAY8:   format = FORMAT_GRAY; break;
        case AV_PIX_FMT_GBRP:    format = FORMAT_RGBP_SEPARATE; break;
        default: printf("unsupported av_pix_format %d\n", avformat); return -1;
    }

    return format;
}

static void hwframe_get_plane_size(AVHWFramesContext* src_frm_ctx, AVBmCodecFrame hw_frame, AVFrame frame, int plane_size[3])
{
    switch (src_frm_ctx->sw_format)
    {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
        plane_size[2] = hw_frame.linesize[2] * frame.height / 2;
    case AV_PIX_FMT_NV12:
        plane_size[1] = hw_frame.linesize[1] * frame.height / 2;
        plane_size[0] = hw_frame.linesize[0] * frame.height;
        break;
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUVJ422P:
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUVJ444P:
        plane_size[2] = hw_frame.linesize[2] * frame.height;
    case AV_PIX_FMT_NV16:
        plane_size[1] = hw_frame.linesize[1] * frame.height;
    case AV_PIX_FMT_GRAY8:
        plane_size[0] = hw_frame.linesize[0] * frame.height;
        break;
    default:
        av_log(NULL, AV_LOG_ERROR, "bmformat %d is not supported!\n", frame.format);
    }

    if(frame.channel_layout == 101)
    {
        plane_size[0] = frame.width * frame.height;
        plane_size[1] = plane_size[2] = frame.width * frame.height / 4;
    }
    return;
}


static void get_plane_size(AVFrame frame, int plane_size[3])
{
    switch (frame.format)
    {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
        plane_size[2] = frame.linesize[2] * frame.height / 2;
    case AV_PIX_FMT_NV12:
        plane_size[1] = frame.linesize[1] * frame.height / 2;
        plane_size[0] = frame.linesize[0] * frame.height;
        break;
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUVJ422P:
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUVJ444P:
        plane_size[2] = frame.linesize[2] * frame.height;
    case AV_PIX_FMT_NV16:
        plane_size[1] = frame.linesize[1] * frame.height;
    case AV_PIX_FMT_GRAY8:
        plane_size[0] = frame.linesize[0] * frame.height;
        break;
    default:
        av_log(NULL, AV_LOG_ERROR, "bmformat %d is not supported!\n", frame.format);
    }

    if(frame.channel_layout == 101)
    {
        plane_size[0] = frame.width * frame.height;
        plane_size[1] = plane_size[2] = frame.width * frame.height / 4;
    }
    return;
}

static void buffer_free(void *opaque, uint8_t *data)
{
    int i;

    if(opaque == NULL){
        printf("parameter error\n");
    }

    buffer_t *buffer = (buffer_t *)opaque;

    if(buffer->host_mem) {
        av_free(buffer->host_mem);
        buffer->host_mem = NULL;
    }

    if(buffer->img)
        bm_image_destroy(*(buffer->img));

    if(buffer->img){
        av_free(buffer->img);
        buffer->img =NULL;
    }

    free(buffer);
    buffer = NULL;

    return;
}

static void buffer_free2(void *opaque, uint8_t *data)
{
    return;
}

static int hwavframe_to_bm_image(bm_handle_t bm_handle, AVFrame in, bm_image *out)
{
    bm_image_format_ext bm_format;
    int size[3] = {0};
    int stride[3] = {0};
    bm_device_mem_t input_addr[3] = {0};
    AVBmCodecFrame *hwsrc = (AVBmCodecFrame*)in.data[4];
    AVHWFramesContext* src_frm_ctx = (AVHWFramesContext*)in.hw_frames_ctx->data;

    hwframe_get_plane_size(src_frm_ctx, *hwsrc, in, size);

    stride[0] = hwsrc->linesize[0];
    stride[1] = hwsrc->linesize[1];
    stride[2] = hwsrc->linesize[2];

    bm_format = (bm_image_format_ext)map_avformat_to_bmformat(src_frm_ctx->sw_format);
    bm_image_create (bm_handle, in.height, in.width, bm_format, DATA_TYPE_EXT_1N_BYTE, out, stride);

    input_addr[0] = bm_mem_from_device((unsigned long long)hwsrc->data[0], size[0]);
    input_addr[1] = bm_mem_from_device((unsigned long long)hwsrc->data[1], size[1]);
    input_addr[2] = bm_mem_from_device((unsigned long long)hwsrc->data[2], size[2]);
    bm_image_attach(*out, input_addr);

    return BM_SUCCESS;
}

static int avframe_to_bm_image(bm_handle_t bm_handle, AVFrame in, bm_image *out)
{
    bm_image_format_ext bm_format;
    int size[3] = {0};
    int stride[3] = {0};
    bm_device_mem_t input_addr[3] = {0};

    stride[0] = in.linesize[4];
    stride[1] = in.linesize[5];
    stride[2] = in.linesize[6];

    get_plane_size(in, size);

    bm_format = (bm_image_format_ext)map_avformat_to_bmformat(in.format);
    bm_image_create (bm_handle, in.height, in.width, bm_format, DATA_TYPE_EXT_1N_BYTE, out, stride);

    input_addr[0] = bm_mem_from_device((unsigned long long)in.data[4], size[0]);
    input_addr[1] = bm_mem_from_device((unsigned long long)in.data[5], size[1]);
    input_addr[2] = bm_mem_from_device((unsigned long long)in.data[6], size[2]);
    bm_image_attach(*out, input_addr);

    return BM_SUCCESS;
}

static int overlay_bm_query_formats_hwaccel(AVFilterContext *ctx)
{
    OverlayBmContext *s = ctx->priv;
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

static int overlay_bm_query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats;
    enum AVPixelFormat pix_fmt;
    int ret;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (ctx->hw_device_ctx != NULL) {
        return overlay_bm_query_formats_hwaccel(ctx);
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
        if ((ret = ff_formats_ref(formats, &ctx->inputs[0]->out_formats)) < 0)
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
        if ((ret = ff_formats_ref(formats, &ctx->outputs[0]->in_formats)) < 0)
            return ret;
    }

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;

}

static int av_frame_convert(bm_handle_t handle, AVFrame *src, AVFrame **dst)
{
    int i;
    int plane_size[3] = {0};
    int stride[4] = {0};
    buffer_t *bufs = NULL;
    bm_image *convert_image = NULL, compressed_image;
    bm_device_mem_t input_addr[4];
    bm_image_format_ext image_format;
    AVFrame *convert_frame = NULL;

    if(src->data[4] && src->channel_layout != 101)
        return 0;

    bufs = av_malloc(sizeof(buffer_t));
    memset(bufs, 0, sizeof(buffer_t));
    convert_image = av_malloc(sizeof(bm_image));
    memset(convert_image, 0, sizeof(bm_image));
    bufs->img = convert_image;

    get_plane_size(*src, plane_size);
    if(src->channel_layout == 101)
        image_format = FORMAT_YUV420P;
    else
        image_format = (bm_image_format_ext)map_avformat_to_bmformat(src->format);

    convert_frame = *dst = av_frame_alloc();
    convert_frame->format   = src->format;
    convert_frame->width    = src->width;
    convert_frame->height   = src->height;
    convert_frame->channels = src->channels;
    av_frame_copy_props(convert_frame, src);

    bufs->host_mem = (uint8_t*)av_malloc(plane_size[0] + plane_size[1] + plane_size[2]);

    for(i=0; i<3; i++) {
        if(plane_size[i]) {

            if(i == 0) {
                convert_frame->buf[i] = av_buffer_create(bufs->host_mem, plane_size[i], buffer_free, bufs, 0);
                convert_frame->data[i] = bufs->host_mem;
            }
            else {
                convert_frame->buf[i] = av_buffer_create(bufs->host_mem + plane_size[i-1], plane_size[i], buffer_free2, NULL, 0);
                convert_frame->data[i] = bufs->host_mem + plane_size[i-1];
            }

            if(src->channel_layout != 101) {
                stride[i] = src->linesize[i];
            }
        }
    }

    bm_image_create(handle, convert_frame->height, convert_frame->width,
                    image_format, DATA_TYPE_EXT_1N_BYTE, convert_image, stride);
    bm_image_alloc_dev_mem_heap_mask(*convert_image, 4);

    // upload or convert
    if(!src->data[4]) {
        bm_image_copy_host_to_device(*convert_image, (void**)src->data);
    } else {
        bm_image_create (handle, src->height, src->width, FORMAT_COMPRESSED, DATA_TYPE_EXT_1N_BYTE, &compressed_image, NULL);

        input_addr[0] = bm_mem_from_device((unsigned long long)src->data[6], src->height * src->linesize[4]);
        input_addr[1] = bm_mem_from_device((unsigned long long)src->data[4], (src->height / 2) * src->linesize[5]);
        input_addr[2] = bm_mem_from_device((unsigned long long)src->data[7], src->linesize[6]);
        input_addr[3] = bm_mem_from_device((unsigned long long)src->data[5], src->linesize[7]);
        bm_image_attach(compressed_image, input_addr);

        bmcv_rect_t crop_rect = {0, 0, src->width, src->height};
        bmcv_image_vpp_convert(handle, 1, compressed_image, convert_image, &crop_rect, BMCV_INTER_LINEAR);
        bm_image_destroy(compressed_image);
    }

    memset(input_addr, 0, sizeof(input_addr));
    memset(stride, 0, sizeof(stride));
    bm_image_get_device_mem(*convert_image, input_addr);
    bm_image_get_stride(*convert_image, stride);

    for(i=0; i<3; i++) {
        convert_frame->data[i+4] = input_addr[i].u.device.device_addr;
        convert_frame->linesize[i] = convert_frame->linesize[i+4] = stride[i];
    }

    return 0;
}

static int ff_framesync_bm_get_frame(FFFrameSync *fs, unsigned in, AVFrame **rframe,
                            unsigned get)
{
    int ret;
    AVFrame *convert_frame = NULL;
    AVFrame *ori_frame = NULL, *clone_frame = NULL;
    AVFilterContext *avctx = fs->parent;
    OverlayBmContext *ctx = avctx->priv;

    unsigned need_copy = 0, i;
    int64_t pts_next;

    if (!fs->in[in].frame) {
        *rframe = NULL;
        return 0;
    }

    ori_frame = fs->in[in].frame;

    if(!avctx->hw_device_ctx) {
        av_frame_convert(ctx->handle, ori_frame, &convert_frame);
        if(convert_frame) {
            av_frame_free(&ori_frame);
            ori_frame = fs->in[in].frame = convert_frame;
        }
    }

    if (get) {
        pts_next = fs->in[in].have_next ? fs->in[in].pts_next : INT64_MAX;
        for (i = 0; i < fs->nb_in && !need_copy; i++)
            if (i != in && fs->in[i].sync &&
                (!fs->in[i].have_next || fs->in[i].pts_next < pts_next))
                need_copy = 1;
        if (need_copy) {
            if (!(clone_frame = av_frame_clone(ori_frame)))
                return AVERROR(ENOMEM);
            // TODO
            if (!avctx->hw_device_ctx && (ret = av_frame_make_writable(clone_frame)) < 0) {
                av_frame_free(&clone_frame);
                return ret;
            }
            /**
             * when activate func is called, filter will free fs->in[in].frame,
             * so clone_frame can make sure that the lifecycle of origin frame will be
             * managed by the user rather than the filter.
             * */
            fs->in[in].frame = clone_frame;
        } else {
            fs->in[in].frame = NULL;
        }
        fs->frame_ready = 0;
    }

    *rframe = ori_frame;
    return 0;
}

static int ff_framesync_bm_dualinput_get(FFFrameSync *fs, AVFrame **f0, AVFrame **f1)
{
    AVFilterContext *ctx = fs->parent;
    AVFrame *mainpic = NULL, *secondpic = NULL;
    int ret;

    if ((ret = ff_framesync_bm_get_frame(fs, 0, &mainpic,   1)) < 0 ||
        (ret = ff_framesync_bm_get_frame(fs, 1, &secondpic, 0)) < 0) {
        av_frame_free(&mainpic);
        return ret;
    }
    av_assert0(mainpic);
    mainpic->pts = av_rescale_q(fs->pts, fs->time_base, ctx->outputs[0]->time_base);
    if (ctx->is_disabled)
        secondpic = NULL;
    *f0 = mainpic;
    *f1 = secondpic;
    return 0;
}

static int bm_overlay_filtering(OverlayBmContext *ctx, AVFrame* input_main, AVFrame* input_overlay)
{
    int ret;
    int i;
    bm_image input_main_img, input_overlay_img;
    bm_device_mem_t mem[3] = {0};
    bmcv_rect_t src_crop_rect, dst_crop_rect;

    if(ctx->hw_device_ctx) {
        ret = hwavframe_to_bm_image(ctx->handle, *input_main, &input_main_img);
        if(ret < 0)
            goto end;

        ret = hwavframe_to_bm_image(ctx->handle, *input_overlay, &input_overlay_img);
        if(ret < 0)
            goto end;
    } else {
        ret = avframe_to_bm_image(ctx->handle, *input_main, &input_main_img);
        if(ret < 0)
            goto end;

        ret = avframe_to_bm_image(ctx->handle, *input_overlay, &input_overlay_img);
        if(ret < 0)
            goto end;
    }


    src_crop_rect.start_x = 0;
    src_crop_rect.start_y = 0;
    src_crop_rect.crop_w = input_overlay->width;
    src_crop_rect.crop_h = input_overlay->height;

    dst_crop_rect.start_x = ctx->x_position;
    dst_crop_rect.start_y = ctx->y_position;
    dst_crop_rect.crop_w = input_overlay->width;
    dst_crop_rect.crop_h = input_overlay->height;

    ret = bmcv_image_vpp_stitch(ctx->handle, 1, &input_overlay_img, input_main_img, &dst_crop_rect, &src_crop_rect, BMCV_INTER_LINEAR);
    if(ret != 0)
        goto end;

    if(!ctx->hw_device_ctx && !ctx->zero_copy)
    {
        bm_image_get_device_mem(input_main_img, mem);
        for(i=0; i<3; i++) {
            if(input_main->data[i])
                bm_memcpy_d2s(ctx->handle, input_main->data[i], mem[i]);
        }
    }

end:
    bm_image_destroy(input_main_img);
    bm_image_destroy(input_overlay_img);
    return ret;
}

/**
 * Perform blend overlay picture over main picture
 */
static int overlay_bm_blend(FFFrameSync *fs)
{
    int ret;
    int pos = 0;

    AVFilterContext *avctx = fs->parent;
    OverlayBmContext *ctx = avctx->priv;
    AVFilterLink *outlink = avctx->outputs[0];
    AVFilterLink *inlink = avctx->inputs[0];

    AVFrame *input_main, *input_overlay;

    ret = ff_framesync_bm_dualinput_get(fs, &input_main, &input_overlay);
    if (ret < 0)
        return ret;

    if (!input_main)
        return AVERROR_BUG;

    if (!input_overlay)
        return ff_filter_frame(outlink, input_main);

    if (ctx->eval_mode == EVAL_MODE_FRAME) {
        pos = input_main->pkt_pos;
        ctx->var_values[VAR_N] = inlink->frame_count_out;
        ctx->var_values[VAR_T] = input_main->pts == AV_NOPTS_VALUE ?
            NAN : input_main->pts * av_q2d(inlink->time_base);
        ctx->var_values[VAR_POS] = pos == -1 ? NAN : pos;
        ctx->var_values[VAR_OVERLAY_W] = ctx->var_values[VAR_OW] = input_overlay->width;
        ctx->var_values[VAR_OVERLAY_H] = ctx->var_values[VAR_OH] = input_overlay->height;
        ctx->var_values[VAR_MAIN_W   ] = ctx->var_values[VAR_MW] = input_main->width;
        ctx->var_values[VAR_MAIN_H   ] = ctx->var_values[VAR_MH] = input_main->height;

        eval_expr(avctx);

        av_log(avctx, AV_LOG_DEBUG, "n:%f t:%f pos:%f x:%f xi:%d y:%f yi:%d\n",
               ctx->var_values[VAR_N], ctx->var_values[VAR_T], ctx->var_values[VAR_POS],
               ctx->var_values[VAR_X], ctx->x_position,
               ctx->var_values[VAR_Y], ctx->y_position);
    }

    ret = bm_overlay_filtering(ctx, input_main, input_overlay);
    if(ret != 0)
    {
        ff_filter_frame(outlink, input_main);
        return ret;
    }

    return ff_filter_frame(outlink, input_main);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx  = inlink->dst;
    OverlayBmContext  *s = inlink->dst->priv;
    int ret;

    /* Finish the configuration by evaluating the expressions
       now when both inputs are configured. */
    s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = ctx->inputs[MAIN   ]->w;
    s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = ctx->inputs[MAIN   ]->h;
    s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = ctx->inputs[OVERLAY]->w;
    s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = ctx->inputs[OVERLAY]->h;
    s->var_values[VAR_X]   = NAN;
    s->var_values[VAR_Y]   = NAN;
    s->var_values[VAR_N]   = 0;
    s->var_values[VAR_T]   = NAN;
    s->var_values[VAR_POS] = NAN;

    if ((ret = set_expr(&s->x_pexpr, s->x_expr, "x", ctx)) < 0 ||
        (ret = set_expr(&s->y_pexpr, s->y_expr, "y", ctx)) < 0)
        return ret;

    if (s->eval_mode == EVAL_MODE_INIT) {
        eval_expr(ctx);
        av_log(ctx, AV_LOG_VERBOSE, "x:%f xi:%d y:%f yi:%d\n",
               s->var_values[VAR_X], s->x_position,
               s->var_values[VAR_Y], s->y_position);
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *avctx = outlink->src;
    OverlayBmContext *ctx = avctx->priv;
    int ret;

    AVFilterLink *inlink_main = NULL, *inlink_overlay = NULL;
    AVHWFramesContext  *frames_ctx_main = NULL, *frames_ctx_overlay = NULL;

    if(avctx->hw_device_ctx != NULL) {
        inlink_main = avctx->inputs[0];
        inlink_overlay = avctx->inputs[1];
        frames_ctx_main = (AVHWFramesContext*)inlink_main->hw_frames_ctx->data;
        frames_ctx_overlay = (AVHWFramesContext*)inlink_overlay->hw_frames_ctx->data;

        ctx->hw_device_ctx = av_buffer_ref(frames_ctx_main->device_ref);
        if (!ctx->hw_device_ctx)
            return AVERROR(ENOMEM);

        ctx->fs.time_base = inlink_main->time_base;

        outlink->format = frames_ctx_main->sw_format;
        outlink->hw_frames_ctx = av_buffer_ref(inlink_main->hw_frames_ctx);
        if (!outlink->hw_frames_ctx)
            return AVERROR(ENOMEM);

        avctx->outputs[0]->hw_frames_ctx = outlink->hw_frames_ctx;
    }

    if ((ret = ff_framesync_init_dualinput(&ctx->fs, avctx)) < 0)
        return ret;

    outlink->w = avctx->inputs[MAIN]->w;
    outlink->h = avctx->inputs[MAIN]->h;
    outlink->time_base = avctx->inputs[MAIN]->time_base;

    return ff_framesync_configure(&ctx->fs);
}

/**
 * Initialize overlay_bm
 */
static av_cold int overlay_bm_init(AVFilterContext *avctx)
{
    OverlayBmContext* ctx = avctx->priv;
    bm_dev_request(&ctx->handle, ctx->sophon_idx);
    ctx->fs.on_event = &overlay_bm_blend;

    return 0;
}

/**
 * Uninitialize overlay_bm
 */
static av_cold void overlay_bm_uninit(AVFilterContext *avctx)
{
    OverlayBmContext* ctx = avctx->priv;

    bm_dev_free(ctx->handle);
    ff_framesync_uninit(&ctx->fs);

    av_expr_free(ctx->x_pexpr); ctx->x_pexpr = NULL;
    av_expr_free(ctx->y_pexpr); ctx->y_pexpr = NULL;

    if(avctx->hw_device_ctx)
        av_buffer_unref(&ctx->hw_device_ctx);
}

/**
 * Activate overlay_bm
 */
static int overlay_bm_activate(AVFilterContext *avctx)
{
    OverlayBmContext *ctx = avctx->priv;
    return ff_framesync_activate(&ctx->fs);
}

#define OFFSET(x) offsetof(OverlayBmContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption overlay_bm_options[] = {
    { "sophon_idx", "sophon_idx, default 0.", OFFSET(sophon_idx), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 255, FLAGS },
    { "zero_copy", "copy filtered frame data to host, default 1.", OFFSET(zero_copy), AV_OPT_TYPE_INT, { .i64 = 1 }, 0, 255, FLAGS },
    { "x", "set the x expression of overlay", OFFSET(x_expr), AV_OPT_TYPE_STRING, { .str = "0" }, 0, 0, FLAGS },
    { "y", "set the y expression of overlay", OFFSET(y_expr), AV_OPT_TYPE_STRING, { .str = "0" }, 0, 0, FLAGS },
    { "eof_action", "Action to take when encountering EOF from secondary input ",
        OFFSET(fs.opt_eof_action), AV_OPT_TYPE_INT, { .i64 = EOF_ACTION_REPEAT },
        EOF_ACTION_REPEAT, EOF_ACTION_PASS, .flags = FLAGS, "eof_action" },
        { "repeat", "Repeat the previous frame.",   0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_REPEAT }, .flags = FLAGS, "eof_action" },
        { "endall", "End both streams.",            0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_ENDALL }, .flags = FLAGS, "eof_action" },
        { "pass",   "Pass through the main input.", 0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_PASS },   .flags = FLAGS, "eof_action" },
    { "eval", "specify when to evaluate expressions", OFFSET(eval_mode), AV_OPT_TYPE_INT, { .i64 = EVAL_MODE_FRAME }, 0, EVAL_MODE_NB - 1, FLAGS, "eval" },
         { "init",  "eval expressions once during initialization", 0, AV_OPT_TYPE_CONST, { .i64=EVAL_MODE_INIT },  .flags = FLAGS, .unit = "eval" },
         { "frame", "eval expressions per-frame",                  0, AV_OPT_TYPE_CONST, { .i64=EVAL_MODE_FRAME }, .flags = FLAGS, .unit = "eval" },
    { "shortest", "force termination when the shortest input terminates", OFFSET(fs.opt_shortest), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, FLAGS },
    { "repeatlast", "repeat overlay of the last overlay frame", OFFSET(fs.opt_repeatlast), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
    { NULL },
};

FRAMESYNC_DEFINE_CLASS(overlay_bm, OverlayBmContext, fs);

static const AVFilterPad overlay_bm_inputs[] = {
    {
        .name         = "main",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
    {
        .name         = "overlay",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
    },
    { NULL }
};

static const AVFilterPad overlay_bm_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = &config_output,
    },
    { NULL }
};

const AVFilter ff_vf_overlay_bm = {
    .name            = "overlay_bm",
    .description     = NULL_IF_CONFIG_SMALL("Overlay one video on top of another using sophon hw"),
    .priv_size       = sizeof(OverlayBmContext),
    .priv_class      = &overlay_bm_class,
    .init            = &overlay_bm_init,
    .uninit          = &overlay_bm_uninit,
    .activate        = &overlay_bm_activate,
    .query_formats   = overlay_bm_query_formats,
    .inputs          = overlay_bm_inputs,
    .outputs         = overlay_bm_outputs,
    .preinit         = overlay_bm_framesync_preinit,
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
};
