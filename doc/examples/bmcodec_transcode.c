/*
 * Video Acceleration API (video transcoding) transcode sample
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file
 * Intel BMCODEC-accelerated transcoding example.
 *
 * @example bmcodec_transcode.c
 * This example shows how to do BMCODEC-accelerated transcoding.
 * Usage: bmcodec_transcode input_stream codec output_stream
 * e.g: - bmcodec_transcode input.mp4 h264_bmcodec output_h264.mp4
 *      - bmcodec_transcode input.mp4 vp9_bmcodec output_vp9.ivf
 */

#include <stdio.h>
#include <errno.h>

#include <libavutil/hwcontext.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>

static AVFormatContext *ifmt_ctx = NULL, *ofmt_ctx = NULL;
static AVBufferRef *hw_device_ctx = NULL;
static AVCodecContext *decoder_ctx = NULL, *encoder_ctx = NULL;
static int video_stream = -1;
static AVStream *ost;
static int initialized = 0;

static enum AVPixelFormat get_bmcodec_format(AVCodecContext *ctx,
                                           const enum AVPixelFormat *pix_fmts)
{
    const enum AVPixelFormat *p;

    fprintf(stderr, "Try to get HW surface format.\n");

    for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == AV_PIX_FMT_BMCODEC) {
            AVHWFramesContext  *frames_ctx;
            int ret;


            /* create a pool of surfaces to be used by the decoder */
            ctx->hw_frames_ctx = av_hwframe_ctx_alloc(hw_device_ctx);
            if (!ctx->hw_frames_ctx)
                return AV_PIX_FMT_NONE;
            frames_ctx   = (AVHWFramesContext*)ctx->hw_frames_ctx->data;

            frames_ctx->format            = AV_PIX_FMT_BMCODEC;
            frames_ctx->sw_format         = ctx->sw_pix_fmt;
            frames_ctx->width             = FFALIGN(ctx->coded_width,  32);
            frames_ctx->height            = FFALIGN(ctx->coded_height, 32);
            frames_ctx->initial_pool_size = 32;

#if 0
            AVQSVFramesContext *frames_hwctx;
            frames_hwctx = frames_ctx->hwctx;
            frames_hwctx->frame_type = MFX_MEMTYPE_VIDEO_MEMORY_DECODER_TARGET;
#endif

            ret = av_hwframe_ctx_init(ctx->hw_frames_ctx);
            if (ret < 0)
                goto failed;

            av_log(ctx, AV_LOG_ERROR, "Got HW surface format:%d.\n", AV_PIX_FMT_BMCODEC);

            return AV_PIX_FMT_BMCODEC;
        }
    }

failed:
    fprintf(stderr, "Unable to decode this file using BMCODEC.\n");
    return AV_PIX_FMT_NONE;
}

static int open_input_file(const char *filename)
{
    int ret;
    AVCodec *decoder = NULL;
    AVStream *video = NULL;

    if ((ret = avformat_open_input(&ifmt_ctx, filename, NULL, NULL)) < 0) {
        fprintf(stderr, "Cannot open input file '%s', Error code: %s\n",
                filename, av_err2str(ret));
        return ret;
    }

    if ((ret = avformat_find_stream_info(ifmt_ctx, NULL)) < 0) {
        fprintf(stderr, "Cannot find input stream information. Error code: %s\n",
                av_err2str(ret));
        return ret;
    }

    ret = av_find_best_stream(ifmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
    if (ret < 0) {
        fprintf(stderr, "Cannot find a video stream in the input file. "
                "Error code: %s\n", av_err2str(ret));
        return ret;
    }
    video_stream = ret;

    if (!(decoder_ctx = avcodec_alloc_context3(decoder)))
        return AVERROR(ENOMEM);

    video = ifmt_ctx->streams[video_stream];
    if ((ret = avcodec_parameters_to_context(decoder_ctx, video->codecpar)) < 0) {
        fprintf(stderr, "avcodec_parameters_to_context error. Error code: %s\n",
                av_err2str(ret));
        return ret;
    }

    decoder_ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
    if (!decoder_ctx->hw_device_ctx) {
        fprintf(stderr, "A hardware device reference create failed.\n");
        return AVERROR(ENOMEM);
    }
    decoder_ctx->get_format = get_bmcodec_format;
    decoder_ctx->framerate.num  = 25;
    decoder_ctx->framerate.den  = 1;

    if ((ret = avcodec_open2(decoder_ctx, decoder, NULL)) < 0)
        fprintf(stderr, "Failed to open codec for decoding. Error code: %s\n",
                av_err2str(ret));

    return ret;
}

static int encode_write(AVFrame *frame)
{
    int ret = 0;
    AVPacket enc_pkt;

    av_init_packet(&enc_pkt);
    enc_pkt.data = NULL;
    enc_pkt.size = 0;

    if ((ret = avcodec_send_frame(encoder_ctx, frame)) < 0) {
        fprintf(stderr, "Error during encoding. Error code: %s\n", av_err2str(ret));
        goto end;
    }
    while (1) {
        ret = avcodec_receive_packet(encoder_ctx, &enc_pkt);
        if (ret)
            break;

        enc_pkt.stream_index = 0;
        av_packet_rescale_ts(&enc_pkt, ifmt_ctx->streams[video_stream]->time_base,
                             ofmt_ctx->streams[0]->time_base);
        ret = av_interleaved_write_frame(ofmt_ctx, &enc_pkt);
        if (ret < 0) {
            fprintf(stderr, "Error during writing data to output file. "
                    "Error code: %s\n", av_err2str(ret));
            return -1;
        }
    }

end:
    if (ret == AVERROR_EOF)
        return 0;
    ret = ((ret == AVERROR(EAGAIN)) ? 0:-1);
    return ret;
}

static int dec_enc(AVPacket *pkt, AVCodec *enc_codec)
{
    AVFrame *frame;
    int ret = 0;

    ret = avcodec_send_packet(decoder_ctx, pkt);
    if (ret < 0) {
        fprintf(stderr, "Error during decoding. Error code: %s\n", av_err2str(ret));
        return ret;
    }

    while (ret >= 0) {
        if (!(frame = av_frame_alloc()))
            return AVERROR(ENOMEM);

        ret = avcodec_receive_frame(decoder_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            av_frame_free(&frame);
            return 0;
        } else if (ret < 0) {
            fprintf(stderr, "Error while decoding. Error code: %s\n", av_err2str(ret));
            goto fail;
        }

        if (!initialized) {
            printf("decoder_ctx->hw_frames_ctx=%p\n", decoder_ctx->hw_frames_ctx);
            /* we need to ref hw_frames_ctx of decoder to initialize encoder's codec.
               Only after we get a decoded frame, can we obtain its hw_frames_ctx */
            encoder_ctx->hw_frames_ctx = av_buffer_ref(decoder_ctx->hw_frames_ctx);
            if (!encoder_ctx->hw_frames_ctx) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            printf("encoder_ctx->hw_frames_ctx=%p\n", encoder_ctx->hw_frames_ctx);

            /* set AVCodecContext Parameters for encoder, here we keep them stay
             * the same as decoder.
             * xxx: now the sample can't handle resolution change case.
             */
            encoder_ctx->time_base = av_inv_q(decoder_ctx->framerate);
            encoder_ctx->pix_fmt   = AV_PIX_FMT_BMCODEC;
            encoder_ctx->width     = decoder_ctx->width;
            encoder_ctx->height    = decoder_ctx->height;

            if ((ret = avcodec_open2(encoder_ctx, enc_codec, NULL)) < 0) {
                fprintf(stderr, "Failed to open encode codec. Error code: %s\n",
                        av_err2str(ret));
                goto fail;
            }

            if (!(ost = avformat_new_stream(ofmt_ctx, enc_codec))) {
                fprintf(stderr, "Failed to allocate stream for output format.\n");
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            ost->time_base = encoder_ctx->time_base;
            ret = avcodec_parameters_from_context(ost->codecpar, encoder_ctx);
            if (ret < 0) {
                fprintf(stderr, "Failed to copy the stream parameters. "
                        "Error code: %s\n", av_err2str(ret));
                goto fail;
            }

            /* write the stream header */
            if ((ret = avformat_write_header(ofmt_ctx, NULL)) < 0) {
                fprintf(stderr, "Error while writing stream header. "
                        "Error code: %s\n", av_err2str(ret));
                goto fail;
            }

            //ret = AVERROR(ENOMEM);
            //goto fail;
            initialized = 1;
        }

        if ((ret = encode_write(frame)) < 0)
            fprintf(stderr, "Error during encoding and writing.\n");

fail:
        av_frame_free(&frame);
        if (ret < 0)
            return ret;
    }
    return 0;
}

int main(int argc, char **argv)
{
    int ret = 0;
    AVPacket dec_pkt;
    AVCodec *enc_codec;

    if (argc != 4) {
        fprintf(stderr, "Usage: %s <input file> <encode codec> <output file>\n"
                "The output format is guessed according to the file extension.\n"
                "\n", argv[0]);
        return -1;
    }

    av_log_set_level(AV_LOG_TRACE);

    ret = av_hwdevice_ctx_create(&hw_device_ctx, AV_HWDEVICE_TYPE_BMCODEC, NULL, NULL, 0);
    if (ret < 0) {
        fprintf(stderr, "Failed to create a BMCODEC device. Error code: %s\n", av_err2str(ret));
        return -1;
    }

    if ((ret = open_input_file(argv[1])) < 0)
        goto end;

    if (!(enc_codec = avcodec_find_encoder_by_name(argv[2]))) {
        fprintf(stderr, "Could not find encoder '%s'\n", argv[2]);
        ret = -1;
        goto end;
    }

    if ((ret = (avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, argv[3]))) < 0) {
        fprintf(stderr, "Failed to deduce output format from file extension. Error code: "
                "%s\n", av_err2str(ret));
        goto end;
    }

    if (!(encoder_ctx = avcodec_alloc_context3(enc_codec))) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = avio_open(&ofmt_ctx->pb, argv[3], AVIO_FLAG_WRITE);
    if (ret < 0) {
        fprintf(stderr, "Cannot open output file. "
                "Error code: %s\n", av_err2str(ret));
        goto end;
    }

    /* read all packets and only transcoding video */
    while (ret >= 0) {
        if ((ret = av_read_frame(ifmt_ctx, &dec_pkt)) < 0)
            break;

        if (video_stream == dec_pkt.stream_index)
            ret = dec_enc(&dec_pkt, enc_codec);

        av_packet_unref(&dec_pkt);
    }

    /* flush decoder */
    dec_pkt.data = NULL;
    dec_pkt.size = 0;
    ret = dec_enc(&dec_pkt, enc_codec);
    av_packet_unref(&dec_pkt);

    /* flush encoder */
    ret = encode_write(NULL);

    /* write the trailer for output stream */
    av_write_trailer(ofmt_ctx);

end:
    avformat_close_input(&ifmt_ctx);
    avformat_close_input(&ofmt_ctx);
    avcodec_free_context(&decoder_ctx);
    avcodec_free_context(&encoder_ctx);
    av_buffer_unref(&hw_device_ctx);
    return ret;
}
