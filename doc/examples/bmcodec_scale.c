/*
 * Video Acceleration API scale sample
 *
 * Copyright (c) 2020, Bitmain Technologies Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * bmcodec-accelerated transcoding example.
 *
 * @example bmcodec_scale.c
 * This example shows how to do bmcodec-accelerated transcoding.
 *
 * Usage: bmcodec_scale input_stream output_stream
 *        bmcodec_scale <sophon device index> input_stream output_stream
 *        bmcodec_scale <sophon device index> <input file> <input format> <output file>
 * e.g: - bmcodec_scale input.264 output.264
 *        bmcodec_scale 1 input.264 output.264
 *        bmcodec_scale 1 input.264 h264 output.264
 *        bmcodec_scale 1 input.265 hevc output.264
 *        bmcodec_scale 1 input.mp4 mp4 output.264
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#include <libavutil/hwcontext.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/timestamp.h>

#define JPEG_XCODING
#define VIDEO_XCODING

#define REDUCE_FRAME_RATE

/* Two-level scaling algorithm */
#ifdef REDUCE_FRAME_RATE
#define FILTER_DESCR "scale_bm=704:576,scale_bm=352:288,fps=fps=25"
#else
#define FILTER_DESCR "scale_bm=704:576,scale_bm=352:288"
#endif

#define FILTER_DESCR2 "scale_bm=1280:720:format=yuvj420p"


typedef struct {
    const char*      sophon_device;
    AVBufferRef*     hw_device_ctx;

    AVFormatContext* ifmt_ctx;
    AVIOContext*     in_avio_ctx;
    AVCodecContext*  decoder_ctx;
    int              video_stream;
    AVStream*        ist;

    AVFilterGraph*   filter_graph;
    AVFilterContext* buffersrc_ctx;
    AVFilterContext* buffersink_ctx;
    AVFrame*         filt_frame;

    AVFormatContext* ofmt_ctx;
    AVIOContext*     out_avio_ctx;
    AVCodec*         encoder;
    AVCodecContext*  encoder_ctx;


    AVFormatContext* jpeg_ofmt_ctx;
    AVIOContext*     jpeg_out_avio_ctx;
    AVCodec*         jpeg_encoder;
    AVCodecContext*  jpeg_enc_ctx;

    AVFilterGraph*   jpeg_filter_graph;
    AVFilterContext* jpeg_buffersrc_ctx;
    AVFilterContext* jpeg_buffersink_ctx;
    AVFrame*         jpeg_filt_frame;
    int              jpeg_out_width;
    int              jpeg_out_height;

    int              initialized;
    int              screenshot;
    int              stopped;

    int              bitrate; // bps
    int              gop_size;
    int              out_width;
    int              out_height;

    int              frame_number;

#ifdef REDUCE_FRAME_RATE
    /* predicted dts of the next packet read for this stream or (when there are
     * several frames in a packet) of the next frame in current packet (in AV_TIME_BASE units) */
    int64_t          next_dts;
    int64_t          dts;       ///< dts of the last packet read for this stream (in AV_TIME_BASE units)

    int64_t          next_pts;  ///< synthetic pts for the next decode frame (in AV_TIME_BASE units)
    int64_t          pts;       ///< current pts of the decoded frame  (in AV_TIME_BASE units)
#endif
} hw_xcode_t;

static hw_xcode_t* xcoder = NULL;


/* TODO modify the callback function according to your needs */
static int cb_read_packet(void *opaque, uint8_t *buf, int buf_size)
{
    FILE* infile = (FILE*)opaque;
    int size = 0;

    if (infile == NULL) {
        return AVERROR_UNKNOWN;
    }

    if (feof(infile)) {
        av_log(NULL, AV_LOG_INFO, "\nEOF of input avio\n");
        return AVERROR_EOF;
    }

    size = fread(buf, 1, buf_size, infile);

    return size;
}

/* TODO modify the callback function according to your needs */
static int cb_write_packet(void *opaque, uint8_t *data, int len)
{
    FILE* outfile = (FILE*)opaque;
    int size = 0;

    if (outfile == NULL) {
        return AVERROR_UNKNOWN;
    }

    if (len <= 0) {
        return 0;
    }

    size = fwrite(data, 1, len, outfile);

    //av_log(NULL, AV_LOG_ERROR, "output bitstream data: %dB\n", len);

    return size;
}

static enum AVPixelFormat get_bmcodec_format(AVCodecContext *ctx,
                                             const enum AVPixelFormat *pix_fmts)
{
    const enum AVPixelFormat *p;

    av_log(ctx, AV_LOG_TRACE, "[%s,%d] Try to get HW surface format.\n", __func__, __LINE__);

    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_BMCODEC) {
            AVHWFramesContext  *frames_ctx;
            int ret;

            /* create a pool of surfaces to be used by the decoder */
            ctx->hw_frames_ctx = av_hwframe_ctx_alloc(ctx->hw_device_ctx);
            if (!ctx->hw_frames_ctx)
                return AV_PIX_FMT_NONE;
            frames_ctx   = (AVHWFramesContext*)ctx->hw_frames_ctx->data;

            frames_ctx->format            = AV_PIX_FMT_BMCODEC;
            frames_ctx->sw_format         = ctx->sw_pix_fmt;

            if (ctx->coded_width > 0)
                frames_ctx->width         = FFALIGN(ctx->coded_width, 32);
            else if (ctx->width > 0)
                frames_ctx->width         = FFALIGN(ctx->width, 32);
            else
                frames_ctx->width         = FFALIGN(1920, 32);

            if (ctx->coded_height > 0)
                frames_ctx->height        = FFALIGN(ctx->coded_height, 32);
            else if (ctx->height > 0)
                frames_ctx->height        = FFALIGN(ctx->height, 32);
            else
                frames_ctx->height        = FFALIGN(1088, 32);

            frames_ctx->initial_pool_size = 0; // Don't prealloc pool.

            ret = av_hwframe_ctx_init(ctx->hw_frames_ctx);
            if (ret < 0)
                goto failed;

            av_log(ctx, AV_LOG_TRACE, "[%s,%d] Got HW surface format:%s.\n",
                   __func__, __LINE__, av_get_pix_fmt_name(AV_PIX_FMT_BMCODEC));

            return AV_PIX_FMT_BMCODEC;
        }
    }

failed:
    av_log(ctx, AV_LOG_ERROR, "Unable to decode this file using BMCODEC.\n");
    return AV_PIX_FMT_NONE;
}

#ifdef JPEG_XCODING
static int jpeg_init_filters(const char *filters_descr)
{
    char args[512];
    AVFilterInOut *inputs = NULL;
    AVFilterInOut *outputs = NULL;
    AVRational time_base = xcoder->ist->time_base; // TODO
    enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_BMCODEC, AV_PIX_FMT_NONE };
    AVBufferSrcParameters par = {0};
    int i, ret = 0;

    xcoder->jpeg_filter_graph = avfilter_graph_alloc();
    if (!xcoder->jpeg_filter_graph) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    xcoder->jpeg_filter_graph->thread_type = 0;

    /* buffer video source: the decoded frames from the decoder will be inserted here. */
    snprintf(args, sizeof(args),
            "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
            xcoder->decoder_ctx->width, xcoder->decoder_ctx->height, xcoder->decoder_ctx->pix_fmt,
            time_base.num, time_base.den,
            xcoder->decoder_ctx->sample_aspect_ratio.num, xcoder->decoder_ctx->sample_aspect_ratio.den);

    ret = avfilter_graph_create_filter(&xcoder->jpeg_buffersrc_ctx,
                                       avfilter_get_by_name("buffer"),
                                       "in",
                                       args, NULL, xcoder->jpeg_filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create buffer source\n");
        goto end;
    }

    par.format = AV_PIX_FMT_NONE;
    par.hw_frames_ctx = xcoder->decoder_ctx->hw_frames_ctx;

    ret = av_buffersrc_parameters_set(xcoder->jpeg_buffersrc_ctx, &par);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_buffersrc_parameters_set failed\n");
        goto end;
    }

    /* buffer video sink: to terminate the filter chain. */
    ret = avfilter_graph_create_filter(&xcoder->jpeg_buffersink_ctx,
                                       avfilter_get_by_name("buffersink"),
                                       "out",
                                       NULL, NULL, xcoder->jpeg_filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create buffer sink\n");
        goto end;
    }
    ret = av_opt_set_int_list(xcoder->jpeg_buffersink_ctx, "pix_fmts", pix_fmts,
                              AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot set output pixel format\n");
        goto end;
    }

    ret = avfilter_graph_parse2(xcoder->jpeg_filter_graph, filters_descr,
                                &inputs, &outputs);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_graph_parse2 failed\n");
        goto end;
    }

    for (i = 0; i < xcoder->jpeg_filter_graph->nb_filters; i++) {
        xcoder->jpeg_filter_graph->filters[i]->hw_device_ctx = av_buffer_ref(xcoder->hw_device_ctx);
        if (!xcoder->jpeg_filter_graph->filters[i]->hw_device_ctx) {
            av_log(NULL, AV_LOG_ERROR, "av_buffer_ref failed\n");
            ret = AVERROR(ENOMEM);
            goto end;
        }
    }

    ret = avfilter_link(xcoder->jpeg_buffersrc_ctx, 0, inputs->filter_ctx, inputs->pad_idx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_link failed\n");
        goto end;
    }

    ret = avfilter_link(outputs->filter_ctx, outputs->pad_idx, xcoder->jpeg_buffersink_ctx, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_link failed\n");
        goto end;
    }

    ret = avfilter_graph_config(xcoder->jpeg_filter_graph, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_graph_config failed\n");
        goto end;
    }

end:
    avfilter_inout_free(&inputs);
    avfilter_inout_free(&outputs);

    return ret;
}

static int jpeg_init_encoder(AVCodec *encoder)
{
    AVCodecContext *enc_ctx = xcoder->jpeg_enc_ctx;
    AVHWFramesContext *frames_ctx;
    AVStream *ost;
    AVDictionary* opts = NULL;
#ifdef REDUCE_FRAME_RATE
    AVRational framerate = {25, 1}; // output fps
#endif
    int ret;

    /* create a pool of surfaces to be used by the decoder */
    enc_ctx->hw_frames_ctx = av_hwframe_ctx_alloc(xcoder->hw_device_ctx);
    if (!enc_ctx->hw_frames_ctx)
        return AV_PIX_FMT_NONE;

    frames_ctx = (AVHWFramesContext*)enc_ctx->hw_frames_ctx->data;

    frames_ctx->format        = AV_PIX_FMT_BMCODEC;
    frames_ctx->sw_format     = AV_PIX_FMT_YUVJ420P;
    frames_ctx->width         = FFALIGN(xcoder->jpeg_out_width, 32);
    frames_ctx->height        = FFALIGN(xcoder->jpeg_out_height, 32);
    frames_ctx->initial_pool_size = 0; // Don't prealloc pool.

    ret = av_hwframe_ctx_init(enc_ctx->hw_frames_ctx);
    if (ret < 0)
        return ret;

    /* set AVCodecContext Parameters for encoder. */
    enc_ctx->pix_fmt   = AV_PIX_FMT_BMCODEC;
    enc_ctx->width     = xcoder->jpeg_out_width;
    enc_ctx->height    = xcoder->jpeg_out_height;
    /* video time_base can be set to whatever is handy and supported by encoder */
#ifdef REDUCE_FRAME_RATE
    enc_ctx->time_base = av_inv_q(framerate); // TODO
    enc_ctx->framerate = framerate;
#else
    enc_ctx->time_base = av_inv_q(xcoder->decoder_ctx->framerate);
    enc_ctx->framerate = xcoder->decoder_ctx->framerate;
#endif

    ret = avcodec_open2(xcoder->jpeg_enc_ctx, encoder, &opts);
    if (ret < 0) {
        av_dict_free(&opts);
        av_log(NULL, AV_LOG_ERROR, "Failed to open encode codec. Error code: %s\n",
               av_err2str(ret));
        return ret;
    }
    av_dict_free(&opts);

    ost = avformat_new_stream(xcoder->jpeg_ofmt_ctx, encoder);
    if (!ost) {
        av_log(NULL, AV_LOG_ERROR, "Failed to allocate stream for output format.\n");
        return AVERROR(ENOMEM);
    }

    ost->time_base = xcoder->jpeg_enc_ctx->time_base;
    ret = avcodec_parameters_from_context(ost->codecpar, xcoder->jpeg_enc_ctx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to copy the stream parameters. "
               "Error code: %s\n", av_err2str(ret));
        return ret;
    }

    /* write the stream header */
    ret = avformat_write_header(xcoder->jpeg_ofmt_ctx, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error while writing stream header. "
               "Error code: %s\n", av_err2str(ret));
        return ret;
    }

    return 0;
}

static int jpeg_encode(AVFrame *frame)
{
    int ret = 0;
    AVPacket enc_pkt;

    av_init_packet(&enc_pkt);
    enc_pkt.data = NULL;
    enc_pkt.size = 0;

    ret = avcodec_send_frame(xcoder->jpeg_enc_ctx, frame);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error during encoding. Error code: %s\n", av_err2str(ret));
        goto end;
    }

    while (1) {
        ret = avcodec_receive_packet(xcoder->jpeg_enc_ctx, &enc_pkt);
        if (ret)
            break;

        enc_pkt.stream_index = 0;
#ifdef REDUCE_FRAME_RATE
        av_packet_rescale_ts(&enc_pkt, xcoder->jpeg_enc_ctx->time_base,
                             xcoder->jpeg_ofmt_ctx->streams[0]->time_base);
#else
        av_packet_rescale_ts(&enc_pkt, xcoder->ist->time_base,
                             xcoder->jpeg_ofmt_ctx->streams[0]->time_base);
#endif
        ret = av_interleaved_write_frame(xcoder->jpeg_ofmt_ctx, &enc_pkt);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during writing data to output file. "
                    "Error code: %s\n", av_err2str(ret));
            return -1;
        }
    }

    /* write the trailer for output stream */
    av_write_trailer(xcoder->jpeg_ofmt_ctx);

end:
    if (ret == AVERROR_EOF)
        return 0;
    ret = ((ret == AVERROR(EAGAIN)) ? 0:-1);
    return ret;
}

static int jpeg_scaling(AVFrame *frame)
{
    int ret = 0;

    /* push the decoded frame into the filtergraph */
    ret = av_buffersrc_add_frame_flags(xcoder->jpeg_buffersrc_ctx, frame, AV_BUFFERSRC_FLAG_KEEP_REF);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error while feeding the filtergraph\n");
        return ret;
    }

    /* pull filtered frames from the filtergraph */
    while (1) {
        ret = av_buffersink_get_frame(xcoder->jpeg_buffersink_ctx, xcoder->jpeg_filt_frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during filtering: %s\n", av_err2str(ret));
            return ret;
        }

        ret = jpeg_encode(xcoder->jpeg_filt_frame);
        if (xcoder->jpeg_filt_frame)
            av_frame_unref(xcoder->jpeg_filt_frame);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during encoding and writing: %s\n", av_err2str(ret));
            return ret;
        }
    }

    return 0;
}
#endif

#ifdef VIDEO_XCODING
static int video_init_filters(const char *filters_descr)
{
    char args[512];
    AVFilterInOut *inputs = NULL;
    AVFilterInOut *outputs = NULL;
    AVRational time_base = xcoder->ist->time_base; // TODO
    enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_BMCODEC, AV_PIX_FMT_NONE };
    AVBufferSrcParameters par = {0};
    int i, ret = 0;

    xcoder->filter_graph = avfilter_graph_alloc();
    if (!xcoder->filter_graph) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    xcoder->filter_graph->thread_type = 0;

    /* buffer video source: the decoded frames from the decoder will be inserted here. */
    snprintf(args, sizeof(args),
            "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
            xcoder->decoder_ctx->width, xcoder->decoder_ctx->height, xcoder->decoder_ctx->pix_fmt,
            time_base.num, time_base.den,
            xcoder->decoder_ctx->sample_aspect_ratio.num, xcoder->decoder_ctx->sample_aspect_ratio.den);

    ret = avfilter_graph_create_filter(&xcoder->buffersrc_ctx,
                                       avfilter_get_by_name("buffer"),
                                       "in",
                                       args, NULL, xcoder->filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create buffer source\n");
        goto end;
    }

    par.format = AV_PIX_FMT_NONE;
    par.hw_frames_ctx = xcoder->decoder_ctx->hw_frames_ctx;

    ret = av_buffersrc_parameters_set(xcoder->buffersrc_ctx, &par);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_buffersrc_parameters_set failed\n");
        goto end;
    }

    /* buffer video sink: to terminate the filter chain. */
    ret = avfilter_graph_create_filter(&xcoder->buffersink_ctx,
                                       avfilter_get_by_name("buffersink"),
                                       "out",
                                       NULL, NULL, xcoder->filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create buffer sink\n");
        goto end;
    }
    ret = av_opt_set_int_list(xcoder->buffersink_ctx, "pix_fmts", pix_fmts,
                              AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot set output pixel format\n");
        goto end;
    }

    ret = avfilter_graph_parse2(xcoder->filter_graph, filters_descr,
                                &inputs, &outputs);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_graph_parse2 failed\n");
        goto end;
    }

    for (i = 0; i < xcoder->filter_graph->nb_filters; i++) {
        xcoder->filter_graph->filters[i]->hw_device_ctx = av_buffer_ref(xcoder->hw_device_ctx);
        if (!xcoder->filter_graph->filters[i]->hw_device_ctx) {
            av_log(NULL, AV_LOG_ERROR, "av_buffer_ref failed\n");
            ret = AVERROR(ENOMEM);
            goto end;
        }
    }

    ret = avfilter_link(xcoder->buffersrc_ctx, 0, inputs->filter_ctx, inputs->pad_idx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_link failed\n");
        goto end;
    }

    ret = avfilter_link(outputs->filter_ctx, outputs->pad_idx, xcoder->buffersink_ctx, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_link failed\n");
        goto end;
    }

    ret = avfilter_graph_config(xcoder->filter_graph, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avfilter_graph_config failed\n");
        goto end;
    }

end:
    avfilter_inout_free(&inputs);
    avfilter_inout_free(&outputs);

    return ret;
}

static int video_init_encoder(void)
{
    AVCodec*        encoder = NULL;
    AVCodecContext *enc_ctx = NULL;
    AVHWFramesContext *frames_ctx;
    AVStream *ost;
    AVDictionary* opts = NULL;
#ifdef REDUCE_FRAME_RATE
    AVRational framerate = {25, 1}; // output fps
#endif
    const char* enc1 = "h264_bm";
    const char* enc2 = "bmx264";
    const char* enc = NULL;
    int ret;

    /* find h264 hw encoder */
    /* bmx264 - 2 */
    enc = (xcoder->bitrate > 100000) ? enc1: enc2;
    xcoder->encoder = avcodec_find_encoder_by_name(enc);
    if (!xcoder->encoder) {
        av_log(NULL, AV_LOG_ERROR, "Could not find encoder '%s'\n", enc);
        return AVERROR(ENOMEM);
    }
    encoder = xcoder->encoder;

    xcoder->encoder_ctx = avcodec_alloc_context3(encoder);
    if (!xcoder->encoder_ctx) {
        av_log(NULL, AV_LOG_ERROR, "avcodec_alloc_context3 failed\n");
        return AVERROR(ENOMEM);
    }
    enc_ctx = xcoder->encoder_ctx;

    /* create a pool of surfaces to be used by the encoder */
    enc_ctx->hw_frames_ctx = av_hwframe_ctx_alloc(xcoder->hw_device_ctx);
    if (!enc_ctx->hw_frames_ctx)
        return AV_PIX_FMT_NONE;

    frames_ctx = (AVHWFramesContext*)enc_ctx->hw_frames_ctx->data;

    frames_ctx->format        = AV_PIX_FMT_BMCODEC;
    /* The output format of scale_bm is yuv420p, not nv12 */
    frames_ctx->sw_format     = AV_PIX_FMT_YUV420P;
    frames_ctx->width         = FFALIGN(xcoder->out_width, 32);
    frames_ctx->height        = FFALIGN(xcoder->out_height, 32);
    frames_ctx->initial_pool_size = 0; // Don't prealloc pool.

    ret = av_hwframe_ctx_init(enc_ctx->hw_frames_ctx);
    if (ret < 0)
        return ret;

    /* set AVCodecContext Parameters for encoder. */
    enc_ctx->pix_fmt   = AV_PIX_FMT_BMCODEC;
    enc_ctx->width     = xcoder->out_width;
    enc_ctx->height    = xcoder->out_height;
    /* video time_base can be set to whatever is handy and supported by encoder */
#ifdef REDUCE_FRAME_RATE
    enc_ctx->time_base = av_inv_q(framerate); // TODO
    enc_ctx->framerate = framerate;
#else
    enc_ctx->time_base = av_inv_q(xcoder->decoder_ctx->framerate);
    enc_ctx->framerate = xcoder->decoder_ctx->framerate;
#endif

    enc_ctx->bit_rate_tolerance = xcoder->bitrate;
    enc_ctx->bit_rate = (int)xcoder->bitrate;
    enc_ctx->gop_size = xcoder->gop_size;;
    enc_ctx->rc_buffer_size = (int)xcoder->bitrate*2; // the range is [0.01, 3] Sec.

    /* bmx264 - 3 */
    /* set private parameters for bm video encoder */
    if (xcoder->bitrate > 100000)
        av_dict_set(&opts, "enc-params", "gop_preset=2:max_qp=44", 0);
    else {
        // av_dict_set(&opts, "preset", "slow", 0);
        /* for debug: aq-mode=0:psy=0:mbtree=0 */
        av_dict_set(&opts, "enc-params", "force-cfr=1:repeat-headers=1:qpmin=10:qpmax=51:bframes=0:keyint=50:keyint-min=50:rc-lookahead=10:scenecut=0", 0);
    }

    ret = avcodec_open2(enc_ctx, encoder, &opts);
    if (ret < 0) {
        av_dict_free(&opts);
        av_log(NULL, AV_LOG_ERROR, "Failed to open encode codec. Error code: %s\n",
               av_err2str(ret));
        return ret;
    }
    av_dict_free(&opts);

    ost = avformat_new_stream(xcoder->ofmt_ctx, encoder);
    if (!ost) {
        av_log(NULL, AV_LOG_ERROR, "Failed to allocate stream for output format.\n");
        return AVERROR(ENOMEM);
    }

    ost->time_base = enc_ctx->time_base;
    ret = avcodec_parameters_from_context(ost->codecpar, enc_ctx);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to copy the stream parameters. "
               "Error code: %s\n", av_err2str(ret));
        return ret;
    }

    /* write the stream header */
    ret = avformat_write_header(xcoder->ofmt_ctx, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error while writing stream header. "
               "Error code: %s\n", av_err2str(ret));
        return ret;
    }

    return 0;
}

static int video_encode(AVFrame *frame)
{
    int ret = 0;
    AVPacket enc_pkt;

    av_init_packet(&enc_pkt);
    enc_pkt.data = NULL;
    enc_pkt.size = 0;

    ret = avcodec_send_frame(xcoder->encoder_ctx, frame);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error during encoding. Error code: %s\n", av_err2str(ret));
        goto end;
    }
    while (1) {
        ret = avcodec_receive_packet(xcoder->encoder_ctx, &enc_pkt);
        if (ret)
            break;

#ifdef VIDEO_XCODING
        enc_pkt.stream_index = 0;
#ifdef REDUCE_FRAME_RATE
        av_packet_rescale_ts(&enc_pkt, xcoder->encoder_ctx->time_base,
                             xcoder->ofmt_ctx->streams[0]->time_base);
#else
        av_packet_rescale_ts(&enc_pkt, xcoder->ist->time_base,
                             xcoder->ofmt_ctx->streams[0]->time_base);
#endif
        ret = av_interleaved_write_frame(xcoder->ofmt_ctx, &enc_pkt);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during writing data to output file. "
                    "Error code: %s\n", av_err2str(ret));
            return -1;
        }
#endif
    }

    if (frame == NULL) {
        /* write the trailer for output stream */
        av_write_trailer(xcoder->ofmt_ctx);
    }
end:
    if (ret == AVERROR_EOF)
        return 0;
    ret = ((ret == AVERROR(EAGAIN)) ? 0:-1);
    return ret;
}

static int video_scaling(AVFrame *frame)
{
    int ret = 0;

    /* push the decoded frame into the filtergraph */
    ret = av_buffersrc_add_frame_flags(xcoder->buffersrc_ctx, frame, AV_BUFFERSRC_FLAG_KEEP_REF);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error while feeding the filtergraph\n");
        return ret;
    }

    /* pull filtered frames from the filtergraph */
    while (1) {
        ret = av_buffersink_get_frame(xcoder->buffersink_ctx, xcoder->filt_frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during filtering: %s\n", av_err2str(ret));
            return ret;
        }

        ret = video_encode(xcoder->filt_frame);
        if (xcoder->filt_frame)
            av_frame_unref(xcoder->filt_frame);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Error during encoding and writing: %s\n", av_err2str(ret));
            return ret;
        }
    }

    return 0;
}

#ifdef REDUCE_FRAME_RATE
static void video_update_ts(AVPacket* pkt, int repeating, int got_output, int64_t* duration_dts)
{
    if (!repeating || !pkt || got_output) {
        if (pkt->duration) {
            *duration_dts = av_rescale_q(pkt->duration, xcoder->ist->time_base, AV_TIME_BASE_Q);
            av_log(NULL, AV_LOG_DEBUG, "<%s,%d> at %s. duration_dts=%ld.\n",
                   __FILE__, __LINE__, __func__, *duration_dts);
        } else if(xcoder->decoder_ctx->framerate.num != 0 && xcoder->decoder_ctx->framerate.den != 0) {
            int ticks= av_stream_get_parser(xcoder->ist) ? av_stream_get_parser(xcoder->ist)->repeat_pict+1 : xcoder->decoder_ctx->ticks_per_frame;
            *duration_dts = ((int64_t)AV_TIME_BASE * xcoder->decoder_ctx->framerate.den * ticks) / xcoder->decoder_ctx->framerate.num / xcoder->decoder_ctx->ticks_per_frame;
            av_log(NULL, AV_LOG_DEBUG, "<%s,%d> at %s. duration_dts=%ld.\n",
                   __FILE__, __LINE__, __func__, *duration_dts);
        }
        if (xcoder->dts != AV_NOPTS_VALUE && *duration_dts)
            xcoder->next_dts += *duration_dts;
        else
            xcoder->next_dts = AV_NOPTS_VALUE;
    }
}
#endif
#endif

static int transcoding(AVPacket *pkt)
{
    AVFrame *frame;
    int ret = 0;
    AVPacket avpkt = *pkt;
#ifdef REDUCE_FRAME_RATE
    int64_t dts = AV_NOPTS_VALUE;
    int repeating = 0;

    if (pkt->dts != AV_NOPTS_VALUE) {
        xcoder->next_dts =
        xcoder->dts = av_rescale_q(pkt->dts, xcoder->ist->time_base, AV_TIME_BASE_Q);
    }

    xcoder->pts = xcoder->next_pts;
    xcoder->dts = xcoder->next_dts;

    if (xcoder->dts != AV_NOPTS_VALUE) {
        dts = av_rescale_q(xcoder->dts, AV_TIME_BASE_Q, xcoder->ist->time_base);
        av_log(NULL, AV_LOG_DEBUG, "<%s,%d> at %s. xcoder->dts=%ld\n", __FILE__, __LINE__, __func__, xcoder->dts);
        av_log(NULL, AV_LOG_DEBUG, "<%s,%d> at %s. dts=%ld\n", __FILE__, __LINE__, __func__, dts);
    }

    avpkt.dts = dts; // TODO

    av_log(NULL, AV_LOG_DEBUG, "<%s,%d> at %s. avpkt.dts=%ld\n", __FILE__, __LINE__, __func__, avpkt.dts);
#endif

    ret = avcodec_send_packet(xcoder->decoder_ctx, &avpkt);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Error during decoding. Error code: %s\n", av_err2str(ret));
        return ret;
    }

    while (ret >= 0) {
#ifdef REDUCE_FRAME_RATE
        int64_t duration_dts = 0;
        int64_t duration_pts;
        int64_t best_effort_timestamp;
#endif

        frame = av_frame_alloc();
        if (!frame)
            return AVERROR(ENOMEM);

        ret = avcodec_receive_frame(xcoder->decoder_ctx, frame);
        if (ret < 0) {
#if defined(VIDEO_XCODING) && defined(REDUCE_FRAME_RATE)
            video_update_ts(&avpkt, repeating, 0, &duration_dts);
#endif
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                av_frame_free(&frame);
                return 0;
            } else {
                av_log(NULL, AV_LOG_ERROR, "Error while decoding. Error code: %s\n", av_err2str(ret));
                goto fail;
            }
        }

#if defined(VIDEO_XCODING) && defined(REDUCE_FRAME_RATE)
        video_update_ts(&avpkt, repeating, 1, &duration_dts);
#endif

        if (!xcoder->initialized) {
#ifdef VIDEO_XCODING
            ret = video_init_filters(FILTER_DESCR);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "video_init_filters failed\n");
                goto fail;
            }

            ret = video_init_encoder();
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "video_init_encoder failed\n");
                goto fail;
            }
#endif

#ifdef JPEG_XCODING
            ret = jpeg_init_filters(FILTER_DESCR2);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "jpeg_init_filters failed\n");
                goto fail;
            }

            ret = jpeg_init_encoder(xcoder->jpeg_encoder);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "jpeg_init_encoder failed\n");
                goto fail;
            }
#endif

            xcoder->initialized = 1;
        }

        if (xcoder->screenshot == 0) {
#ifdef JPEG_XCODING
            ret = jpeg_scaling(frame);
            if (ret < 0) {
                goto fail;
            }
#endif
            xcoder->screenshot = 1;
        }

#ifdef REDUCE_FRAME_RATE
        best_effort_timestamp = frame->best_effort_timestamp;
        if (best_effort_timestamp != AV_NOPTS_VALUE) {
            int64_t ts;
            frame->pts = best_effort_timestamp;

            ts = av_rescale_q(frame->pts, xcoder->ist->time_base, AV_TIME_BASE_Q);
            if (ts != AV_NOPTS_VALUE)
                xcoder->next_pts = xcoder->pts = ts;
        }

        av_log(NULL, AV_LOG_DEBUG, "<%s,%d> frame pts:%ld. frame pts_time:%s\n",
               __func__, __LINE__, frame->pts, av_ts2timestr(frame->pts, &xcoder->ist->time_base));

        duration_pts = frame->pkt_duration;
        if (duration_pts > 0)
            xcoder->next_pts += av_rescale_q(duration_pts, xcoder->ist->time_base, AV_TIME_BASE_Q);
        else
            xcoder->next_pts += duration_dts;
#else
        frame->pts = frame->best_effort_timestamp;
#endif

#ifdef VIDEO_XCODING
        ret = video_scaling(frame);
        if (ret < 0) {
            goto fail;
        }
        av_log(NULL, AV_LOG_INFO, "frame: %d\r", xcoder->frame_number);
        xcoder->frame_number++;
#endif

        if (frame)
            av_frame_free(&frame);

#ifdef REDUCE_FRAME_RATE
        repeating = 1;
#endif
    }

fail:
    if (frame)
        av_frame_free(&frame);
    if (ret < 0)
        return ret;
    return 0;
}

static void handler(int sig)
{
    xcoder->stopped = 1;
    av_log(NULL, AV_LOG_WARNING, "program will exit\n");
}

static void init_xcoder(hw_xcode_t* xcoder)
{
    if (xcoder == NULL)
        return;

    memset(xcoder, 0, sizeof(hw_xcode_t));


    xcoder->sophon_device  = "0";
    xcoder->hw_device_ctx  = NULL;

    xcoder->ifmt_ctx       = NULL;
    xcoder->in_avio_ctx    = NULL;
    xcoder->decoder_ctx    = NULL;
    xcoder->video_stream   = -1;

    xcoder->ofmt_ctx       = NULL;
    xcoder->out_avio_ctx   = NULL;
    xcoder->encoder_ctx    = NULL;
    xcoder->encoder        = NULL;

    xcoder->filter_graph   = NULL;
    xcoder->buffersrc_ctx  = NULL;
    xcoder->buffersink_ctx = NULL;
    xcoder->filt_frame     = NULL;

    xcoder->jpeg_ofmt_ctx     = NULL;
    xcoder->jpeg_out_avio_ctx = NULL;
    xcoder->jpeg_enc_ctx      = NULL;
    xcoder->jpeg_encoder      = NULL;

    xcoder->jpeg_filter_graph   = NULL;
    xcoder->jpeg_buffersrc_ctx  = NULL;
    xcoder->jpeg_buffersink_ctx = NULL;
    xcoder->jpeg_filt_frame     = NULL;

    xcoder->jpeg_out_width      = 1280;
    xcoder->jpeg_out_height     =  720;

    xcoder->screenshot     = 0;

    xcoder->initialized    = 0;
    xcoder->stopped        = 0;

    xcoder->bitrate        = 256*1000; // bps
    //xcoder->bitrate        = 32*1000; // bps
    xcoder->gop_size       = 64;
    xcoder->out_width      = 352;
    xcoder->out_height     = 288;

    xcoder->frame_number   = 0;

#ifdef REDUCE_FRAME_RATE
    xcoder->dts = AV_NOPTS_VALUE;
    xcoder->pts = AV_NOPTS_VALUE;

    xcoder->next_dts = AV_NOPTS_VALUE;
    xcoder->next_pts = AV_NOPTS_VALUE;
#endif
}

static void deinit_xcoder(hw_xcode_t* xcoder)
{
    if (xcoder == NULL)
        return;

    avformat_close_input(&xcoder->ifmt_ctx);
    if (xcoder->in_avio_ctx) {
        av_freep(&xcoder->in_avio_ctx->buffer);
        av_freep(&xcoder->in_avio_ctx);
    }
#ifdef VIDEO_XCODING
    avformat_free_context(xcoder->ofmt_ctx);
    if (xcoder->out_avio_ctx) {
        av_freep(&xcoder->out_avio_ctx->buffer);
        av_freep(&xcoder->out_avio_ctx);
    }
#endif

#ifdef JPEG_XCODING
    avformat_free_context(xcoder->jpeg_ofmt_ctx);
    if (xcoder->jpeg_out_avio_ctx) {
        av_freep(&xcoder->jpeg_out_avio_ctx->buffer);
        av_freep(&xcoder->jpeg_out_avio_ctx);
    }
    avcodec_free_context(&xcoder->jpeg_enc_ctx);
#endif
    avcodec_free_context(&xcoder->decoder_ctx);
    avcodec_free_context(&xcoder->encoder_ctx);
    av_buffer_unref(&xcoder->hw_device_ctx);

    if (xcoder->filter_graph)
        avfilter_graph_free(&xcoder->filter_graph);

    if (xcoder->jpeg_filter_graph)
        avfilter_graph_free(&xcoder->jpeg_filter_graph);

    if (xcoder->jpeg_filt_frame)
        av_frame_free(&xcoder->jpeg_filt_frame);
    if (xcoder->filt_frame)
        av_frame_free(&xcoder->filt_frame);
}

static int bmx264_set_env_vars(void)
{
    int ret;
    ret = setenv("BMCPU_FIPFILE", "/data/bmcpu/files/fip.bin", 0);
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to set env BMCPU_FIPFILE: %s\n", strerror(errno));
        return -1;
    }
    ret = setenv("BMCPU_RAMBOOTFILE", "/data/bmcpu/files/ramboot_rootfs.itb", 0);
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to set env BMCPU_RAMBOOTFILE: %s\n", strerror(errno));
        return -1;
    }
#if 0
    ret = setenv("BMCPU_LOGFILE", "output/device_logfile.txt", 0);
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to set env BMCPU_LOGFILE: %s\n", strerror(errno));
        return -1;
    }
#endif
    ret = setenv("BMX264_LIBFILE", "/data/bmcpu/files/libx264.so", 0);
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to set env BMX264_LIBFILE: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int bmx264_unset_env_vars(void)
{
    int ret;
    ret = unsetenv("BMCPU_FIPFILE");
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to unset env BMCPU_FIPFILE: %s\n", strerror(errno));
    }
    ret = unsetenv("BMCPU_RAMBOOTFILE");
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to unset env BMCPU_RAMBOOTFILE: %s\n", strerror(errno));
    }
#if 0
    ret = unsetenv("BMCPU_LOGFILE");
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to unset env BMCPU_LOGFILE: %s\n", strerror(errno));
    }
#endif
    ret = unsetenv("BMX264_LIBFILE");
    if (ret==-1) {
        av_log(NULL, AV_LOG_ERROR, "Failed to unset env BMX264_LIBFILE: %s\n", strerror(errno));
    }

    return 0;
}


int main(int argc, char **argv)
{
    AVInputFormat* iformat;
    AVCodec *decoder = NULL;
    AVDictionary* opts = NULL;
    AVPacket dec_pkt = {0};
    uint8_t* in_avio_buf = NULL;
    int in_avio_buf_size = 4096*4;
    uint8_t* out_avio_buf = NULL;
    int out_avio_buf_size = 4096*4;
#ifdef JPEG_XCODING
    uint8_t* jpeg_out_avio_buf = NULL;
    int jpeg_out_avio_buf_size = 4096*4;
#endif
    char* infilename  = NULL;
    char* outfilename = NULL;
    const char* infilefmt   = "h264"; /* h264, hevc, mp4 */
    const char* outjpegname = "screenshot.jpg";
    FILE* infile  = NULL;
    FILE* outfile = NULL;
    FILE* jpegfile = NULL;
    int ret = 0;

    if (argc != 5 && argc != 4 && argc != 3) {
        fprintf(stderr, "Usage:\n"
                "\t%s <input file> <output file>\n"
                "\t%s <sophon device index> <input file> <output file>\n"
                "\t%s <sophon device index> <input file> <input format> <output file>\n"
                "\tThe output format is guessed according to the file extension.\n"
                "\n", argv[0], argv[0], argv[0]);
        return -1;
    }

    signal(SIGINT,  handler);
    signal(SIGTERM, handler);

    xcoder = av_malloc(sizeof(hw_xcode_t));
    if (!xcoder) {
        av_log(NULL, AV_LOG_ERROR, "av_mallocz failed\n");
        return -1;
    }

    init_xcoder(xcoder);

    if (argc == 5) {
        xcoder->sophon_device = argv[1];
        infilename    = argv[2];
        infilefmt     = argv[3];
        outfilename   = argv[4];
    } else if (argc == 4) {
        xcoder->sophon_device = argv[1];
        infilename    = argv[2];
        outfilename   = argv[3];
    } else { /* if (argc == 3) */
        infilename    = argv[1];
        outfilename   = argv[2];
    }

    /* bmx264 - 1 */
    ret = bmx264_set_env_vars();
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to bmx264_set_env_vars\n");
        ret = AVERROR_UNKNOWN;
        goto end;
    }

    xcoder->filt_frame = av_frame_alloc();
    if (!xcoder->filt_frame) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate frame");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    xcoder->jpeg_filt_frame = av_frame_alloc();
    if (!xcoder->jpeg_filt_frame) {
        av_log(NULL, AV_LOG_ERROR, "Could not allocate frame");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = av_hwdevice_ctx_create(&xcoder->hw_device_ctx, AV_HWDEVICE_TYPE_BMCODEC, xcoder->sophon_device, NULL, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to create a BMCODEC device. Error code: %s\n", av_err2str(ret));
        goto end;
    }

    /* set avio for demuxer and decoder */
    in_avio_buf = (uint8_t*)av_mallocz(in_avio_buf_size);
    if (in_avio_buf == NULL) {
        av_log(NULL, AV_LOG_ERROR, "av_mallocz failed\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    infile = fopen(infilename, "rb");
    if (infile == NULL) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open input file: %s\n", infilename);
        ret = -1;
        goto end;
    }

    xcoder->in_avio_ctx = avio_alloc_context(in_avio_buf, in_avio_buf_size, 0,
                                             (void*)infile, cb_read_packet,
                                             NULL, NULL);
    if (xcoder->in_avio_ctx == NULL) {
        av_log(NULL, AV_LOG_ERROR, "avio_alloc_context failed\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    xcoder->ifmt_ctx = avformat_alloc_context();
    if (xcoder->ifmt_ctx == NULL) {
        av_log(NULL, AV_LOG_ERROR, "avformat_alloc_context failed\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }
    xcoder->ifmt_ctx->pb = xcoder->in_avio_ctx;

    /* find hevc demuxer */
#if 0
    ret = av_probe_input_buffer(xcoder->in_avio_ctx, &iformat,
                                NULL, NULL, NULL, 2048); // TODO
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "av_probe_input_buffer failed");
        ret = AVERROR_DEMUXER_NOT_FOUND;
        goto end;
    }
#else
    iformat = av_find_input_format(infilefmt);
    if (iformat == NULL) {
        av_log(NULL, AV_LOG_ERROR, "av_find_input_format(%s) failed", infilefmt);
        ret = AVERROR_DEMUXER_NOT_FOUND;
        goto end;
    }
#endif

    ret = avformat_open_input(&xcoder->ifmt_ctx, NULL, iformat, NULL);
    if (ret != 0) {
        av_log(NULL, AV_LOG_ERROR, "avformat_open_input failed\n");
        goto end;
    }

    // TODO
    ret = avformat_find_stream_info(xcoder->ifmt_ctx, NULL);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot find input stream information. Error code: %s\n",
               av_err2str(ret));
        goto end;
    }

    // TODO
    ret = av_find_best_stream(xcoder->ifmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot find a video stream in the input file. "
               "Error code: %s\n", av_err2str(ret));
        goto end;
    }
    xcoder->video_stream = ret;

    xcoder->decoder_ctx = avcodec_alloc_context3(decoder);
    if (!xcoder->decoder_ctx) {
        av_log(NULL, AV_LOG_ERROR, "avcodec_alloc_context3 failed\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    xcoder->ist = xcoder->ifmt_ctx->streams[xcoder->video_stream];
    ret = avcodec_parameters_to_context(xcoder->decoder_ctx, xcoder->ist->codecpar);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "avcodec_parameters_to_context error. Error code: %s\n",
               av_err2str(ret));
        goto end;
    }

    xcoder->decoder_ctx->hw_device_ctx = av_buffer_ref(xcoder->hw_device_ctx);
    if (!xcoder->decoder_ctx->hw_device_ctx) {
        av_log(NULL, AV_LOG_ERROR, "A hardware device reference create failed.\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    xcoder->decoder_ctx->get_format = get_bmcodec_format;
    xcoder->decoder_ctx->framerate  = xcoder->ist->avg_frame_rate; // TODO
    xcoder->decoder_ctx->time_base  = av_inv_q(xcoder->ist->avg_frame_rate); // TODO

#ifdef REDUCE_FRAME_RATE
    av_log(xcoder->decoder_ctx, AV_LOG_DEBUG, "0: decoder time base: %d/%d.\n",
           xcoder->decoder_ctx->time_base.num,
           xcoder->decoder_ctx->time_base.den);

    av_log(xcoder->decoder_ctx, AV_LOG_DEBUG, "0: input time base: %d/%d.\n",
           xcoder->ist->time_base.num,
           xcoder->ist->time_base.den);

    av_log(xcoder->decoder_ctx, AV_LOG_DEBUG, "1: input frame rate: %d/%d.\n",
           xcoder->ist->avg_frame_rate.num,
           xcoder->ist->avg_frame_rate.den);
    av_log(xcoder->decoder_ctx, AV_LOG_DEBUG, "2: input frame rate: %d/%d.\n",
           xcoder->ist->r_frame_rate.num,
           xcoder->ist->r_frame_rate.den);
#endif

    /* set the private parameters for bm video decoder */
    av_dict_set_int(&opts, "extra_frame_buffer_num", 1, 0);
    av_dict_set_int(&opts, "cbcr_interleave", 0, 0);

    ret = avcodec_open2(xcoder->decoder_ctx, decoder, &opts);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Failed to open codec for decoding. Error code: %s\n",
               av_err2str(ret));
        goto end;
    }
    av_dict_free(&opts);

    /* video encoder */
    outfile = fopen(outfilename, "wb");
    if (outfile == NULL) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open output file: %s\n", outfilename);
        goto end;
    }

#ifdef VIDEO_XCODING
    out_avio_buf = av_malloc(out_avio_buf_size);
    if (!out_avio_buf) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    xcoder->out_avio_ctx = avio_alloc_context(out_avio_buf, out_avio_buf_size, 1,
                                              (void*)outfile, NULL, &cb_write_packet, NULL);
    if (!xcoder->out_avio_ctx) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = avformat_alloc_output_context2(&xcoder->ofmt_ctx, NULL, "h264", NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not create output context\n");
        goto end;
    }
    xcoder->ofmt_ctx->pb = xcoder->out_avio_ctx;
#endif

    /* jpeg encoder */
    /* find jpeg hw encoder */
    xcoder->jpeg_encoder = avcodec_find_encoder_by_name("jpeg_bm");
    if (!xcoder->jpeg_encoder) {
        av_log(NULL, AV_LOG_ERROR, "Could not find encoder 'jpeg_bm'\n");
        ret = -1;
        goto end;
    }
    xcoder->jpeg_enc_ctx = avcodec_alloc_context3(xcoder->jpeg_encoder);
    if (!xcoder->jpeg_enc_ctx) {
        av_log(NULL, AV_LOG_ERROR, "avcodec_alloc_context3 failed\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }

    jpegfile = fopen(outjpegname, "wb");
    if (jpegfile == NULL) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open output jpeg file: %s\n", outjpegname);
        goto end;
    }

#ifdef JPEG_XCODING
    jpeg_out_avio_buf = av_malloc(jpeg_out_avio_buf_size);
    if (!jpeg_out_avio_buf) {
        ret = AVERROR(ENOMEM);
        goto end;
    }
    xcoder->jpeg_out_avio_ctx = avio_alloc_context(jpeg_out_avio_buf, jpeg_out_avio_buf_size, 1,
                                                   (void*)jpegfile, NULL, &cb_write_packet, NULL);
    if (!xcoder->jpeg_out_avio_ctx) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = avformat_alloc_output_context2(&xcoder->jpeg_ofmt_ctx, NULL, "mjpeg", NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not create output context\n");
        goto end;
    }
    xcoder->jpeg_ofmt_ctx->pb = xcoder->jpeg_out_avio_ctx;
#endif

#ifdef REDUCE_FRAME_RATE
    if (xcoder->ist->avg_frame_rate.num)
        xcoder->dts = - xcoder->decoder_ctx->has_b_frames * AV_TIME_BASE / av_q2d(xcoder->ist->avg_frame_rate);
    else
        xcoder->dts = 0;
    xcoder->pts = 0;

    if (xcoder->next_dts == AV_NOPTS_VALUE)
        xcoder->next_dts = xcoder->dts;
    if (xcoder->next_pts == AV_NOPTS_VALUE)
        xcoder->next_pts = xcoder->pts;
#endif

    /* read all packets and only transcoding video */
    while (ret >= 0 && !xcoder->stopped) {
        ret = av_read_frame(xcoder->ifmt_ctx, &dec_pkt);
        if (ret < 0)
            break;

        if (xcoder->video_stream != dec_pkt.stream_index) {
            av_packet_unref(&dec_pkt);
            continue;
        }

        ret = transcoding(&dec_pkt);
        if (ret < 0)
            goto end;

        av_packet_unref(&dec_pkt);
    }

    /* flush decoder */
    dec_pkt.data = NULL;
    dec_pkt.size = 0;
    ret = transcoding(&dec_pkt);
    av_packet_unref(&dec_pkt);

#ifdef VIDEO_XCODING
    /* flush encoder */
    ret = video_encode(NULL);
#endif

end:
    if (xcoder) {
        deinit_xcoder(xcoder);
        av_free(xcoder);
        xcoder = NULL;
    }

    /* bmx264 - 4 */
    ret = bmx264_unset_env_vars();

    if (jpegfile) {
        fclose(jpegfile);
        jpegfile = NULL;
    }
    if (outfile) {
        fclose(outfile);
        outfile = NULL;
    }
    if (infile) {
        fclose(infile);
        infile = NULL;
    }

    return ret;
}
