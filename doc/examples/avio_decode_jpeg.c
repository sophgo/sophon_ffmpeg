
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavcodec/avcodec.h>
#include <libavutil/pixfmt.h>
#include <stdio.h>


//static int bs_cnt = 0;

static int read_buffer(void *opaque, uint8_t *buf, int buf_size)
{
    FILE* infile = (FILE*)opaque;
    int size = 0;

    if (feof(infile))
    {
        printf("EOF of AVIO\n");
        return AVERROR_EOF;
    } 

    size = fread(buf, 1, buf_size, infile);

    //printf("[%d] size: %d\n", bs_cnt, size);
    //bs_cnt++;

    return size;
}

int main(int argc, char* argv[])
{
    AVInputFormat* iformat;
    AVFormatContext* fmt_ctx;
    AVCodecContext* avctx;
    AVCodec* codec;
    AVIOContext* avio_ctx = NULL;
    AVFrame* frame = NULL;
    AVPacket pkt;
    AVDictionary* dict = NULL;
    int got_picture;
    uint8_t* avio_buff;
    const int avio_buf_size = 16*1024;
    char* infilename;
    FILE *infile = NULL;
    void *opaque = NULL;
    int bs_size;
    int buff_size;
    int ret;

    if (argc != 2)
    {
        fprintf(stderr, "Usage:%s <input.jpg>\n", argv[0]);
        return -1;
    }

    infilename  = argv[1];

    infile = fopen(infilename, "rb");
    if (infile == NULL)
    {
        fprintf(stderr, "Failed to open input jpeg file!\n");
        return -1;
    }

    fseek(infile, 0, SEEK_END);
    bs_size = ftell(infile);
    printf("bs size: %d\n", bs_size);
    fseek(infile, 0, SEEK_SET);

    /* set avio */
    avio_buff = (uint8_t *)av_malloc(avio_buf_size);
    if (avio_buff == NULL)
    {
        fprintf(stderr, "av_malloc failed\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    opaque = (void*)infile;
    avio_ctx = avio_alloc_context(avio_buff, avio_buf_size, 0,
                                  opaque, read_buffer, NULL, NULL);
    if (avio_ctx == NULL)
    {
        fprintf(stderr, "avio_alloc_context failed\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    fmt_ctx = avformat_alloc_context();
    if (fmt_ctx == NULL)
    {
        fprintf(stderr, "avformat_alloc_context failed\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    fmt_ctx->pb = avio_ctx;

    /* mjpeg demuxer */
    iformat = av_find_input_format("mjpeg");
    if (iformat == NULL)
    {
        fprintf(stderr, "av_find_input_format failed.\n");
        ret = AVERROR_DEMUXER_NOT_FOUND;
        goto fail;
    }
    ret = avformat_open_input(&fmt_ctx, NULL, iformat, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "Couldn't open input stream.\n");
        goto fail;
    }

    av_dump_format(fmt_ctx, 0, 0, 0); // TODO

    /* jpeg_bm decoder */
    codec = avcodec_find_decoder_by_name("jpeg_bm");
    if (codec == NULL)
    {
        fprintf(stderr, "Codec not found.\n");
        ret = AVERROR_DECODER_NOT_FOUND;
        goto fail;
    }

    avctx = avcodec_alloc_context3(codec);
    if (avctx == NULL)
    {
        fprintf(stderr, "Could not allocate video codec context\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    /* Set parameters for jpeg_bm decoder */

    /* The output of bm jpeg decoder is chroma-separated,for example, YUVJ420P */
    av_dict_set_int(&dict, "chroma_interleave", 0, 0);
    /* The bitstream buffer size (KB) */
#define BS_MASK (1024*16-1)
    buff_size = (bs_size+BS_MASK)&(~BS_MASK); /* in unit of 16k */
#undef  BS_MASK
#define JPU_PAGE_UNIT_SIZE 256 /* each page unit of jpu is 256 byte */
    /* Avoid the false alarm that bs buffer is empty (SA3SW-252) */
    if (buff_size - bs_size < JPU_PAGE_UNIT_SIZE)
        buff_size += 16*1024;  /* in unit of 16k */
#undef JPU_PAGE_UNIT_SIZE
    buff_size /= 1024;
    printf("bs buffer size: %dK\n", buff_size);
    av_dict_set_int(&dict, "bs_buffer_size", buff_size, 0);
    /* Extra frame buffers: "0" for still jpeg, at least "2" for mjpeg */
    av_dict_set_int(&dict, "num_extra_framebuffers", 0, 0);

    ret = avcodec_open2(avctx, codec, &dict);
    if (ret < 0)
    {
        printf("Could not open codec.\n");
        ret = AVERROR_UNKNOWN;
        goto fail;
    }

    frame = av_frame_alloc();
    if (frame == NULL)
    {
        fprintf(stderr, "Could not allocate video frame\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_init_packet(&pkt);
    pkt.data = NULL;
    pkt.size = 0;

    while (1)
    {
        ret = av_read_frame(fmt_ctx, &pkt);
        if (ret == AVERROR_EOF)
        {
            printf("The EOF of bit stream!\n");
            ret = 0;
            break;
        }
        else if (ret < 0)
        {
            fprintf(stderr, "[%s, %d] av_read_frame failed!\n", __FUNCTION__, __LINE__);
            break;
        }

        ret = avcodec_decode_video2(avctx, frame, &got_picture, &pkt);
        if (ret < 0)
        {
            fprintf(stderr, "[%s, %d] Decode Error.\n", __FUNCTION__, __LINE__);
            break;
        }

        if (got_picture)
        {
            printf("frame format: %d\n", frame->format);
            printf("frame width : %d\n", frame->width);
            printf("frame height: %d\n", frame->height);

            /* TODO */
        }

        av_packet_unref(&pkt);
        av_frame_unref(frame);
    }

fail:
    av_packet_unref(&pkt);

    if (frame)
        av_frame_free(&frame);

    avformat_close_input(&fmt_ctx);

    if (avio_ctx)
    {
        av_freep(&avio_ctx->buffer);
        av_freep(&avio_ctx);
    }

    avcodec_close(avctx);

    if (infile)
        fclose(infile);

    if (ret < 0) {
        fprintf(stderr, "Error occurred: %s\n", av_err2str(ret));
        return -1;
    }

    return 0;
}

