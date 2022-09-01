/*
 * Copyright (c) 2012 Stefano Sabatini
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
 * Demuxing and decoding example.
 *
 * Show how to use the libavformat and libavcodec API to demux and
 * decode audio and video data.
 * @example demuxing_decoding.c
 */
#include "config.h"
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
#include <libavutil/timestamp.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include "libavutil/time.h"
#include "libavutil/threadmessage.h"
#include <stdatomic.h>

#include <stdlib.h>

#if HAVE_TERMIOS_H
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#elif HAVE_KBHIT
#include <conio.h>
#endif

#if HAVE_IO_H
#include <io.h>
#endif
#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <time.h>

#if HAVE_PTHREADS
#include <pthread.h>
#endif

#define MAX_INST_NUM 64
typedef struct MultiInstTest {
    AVFormatContext *fmt_ctx;
    AVCodecContext *video_dec_ctx, *audio_dec_ctx;
    int width, height;
    enum AVPixelFormat pix_fmt;
    AVStream *video_stream, *audio_stream;
    const char *src_filename;
    const char *video_dst_filename;
    const char *audio_dst_filename;
    FILE *video_dst_file;
    FILE *audio_dst_file;
    /*
    uint8_t *video_dst_data[4];
    int      video_dst_linesize[4];
    int video_dst_bufsize;
    */
    int video_stream_idx, audio_stream_idx;
    AVFrame *frame;
    AVPacket pkt;
    volatile int video_frame_count;
    int audio_frame_count;
    
    /* Enable or disable frame reference counting. You are not supposed to support
     * both paths in your application but pick the one most appropriate to your
     * needs. Look for the use of refcount in this example to see what are the
     * differences of API usage between them. */
    int refcount;
    volatile int end_of;
    volatile int first_frame_flag;
    volatile int64_t start_time_dec, get_time_dec, last_time_read_pkt;
    volatile double fps_dec;
#if HAVE_THREADS
    AVThreadMessageQueue *in_thread_queue;
    pthread_t thread;           /* thread reading from this file */
    int non_blocking;           /* reading packets from the thread should not block */
    int joined;                 /* the thread has been joined */
    int thread_queue_size;      /* maximum number of queued packets */
#endif
    int inst_idx;
    int first_pkt_flag;
    atomic_uint_fast64_t start_msg_pts, first_pkt_pts, input_pts, output_pts, ave_delay, max_delay, min_delay, total_max_delay, total_min_delay, total_delay;
    atomic_uint_fast32_t ave_num;
} MultiInstTest;

MultiInstTest inst[MAX_INST_NUM] = {0};
int inst_num = 0;
char codec_name[255];
int codec_name_flag = 0;
AVRational pts_to_ms = {1, 1000};

static int decode_packet(MultiInstTest *test_inst, int *got_frame, int cached)
{
    int ret = 0;
    int decoded = test_inst->pkt.size;
    

    *got_frame = 0;

    if (test_inst->pkt.stream_index == test_inst->video_stream_idx) {
        av_log(test_inst->video_dec_ctx, AV_LOG_TRACE, "inst: %d", test_inst->inst_idx);
        /* decode video frame */
        ret = avcodec_decode_video2(test_inst->video_dec_ctx, test_inst->frame, got_frame, &test_inst->pkt);
        if (ret < 0) {
            fprintf(stderr, "Error decoding video frame (%s)\n", av_err2str(ret));
            return ret;
        }

        if (*got_frame) {
#if 0
            if (test_inst->frame->width != test_inst->width || test_inst->frame->height != test_inst->height ||
                test_inst->frame->format != test_inst->pix_fmt) {
                /* To handle this change, one could call av_image_alloc again and
                 * decode the following frames into another rawvideo file. */
                fprintf(stderr, "Error: Width, height and pixel format have to be "
                        "constant in a rawvideo file, but the width, height or "
                        "pixel format of the input video changed:\n"
                        "old: width = %d, height = %d, format = %s\n"
                        "new: width = %d, height = %d, format = %s\n",
                        test_inst->width, test_inst->height, av_get_pix_fmt_name(test_inst->pix_fmt),
                        test_inst->frame->width, test_inst->frame->height,
                        av_get_pix_fmt_name(test_inst->frame->format));
                return -1;
            }
#endif
            if(test_inst->first_frame_flag == 0) {
                test_inst->first_frame_flag = 1;
                test_inst->start_time_dec = av_gettime();
            }
            test_inst->video_frame_count++;
            //printf("frame: %d, pts: %ld, pkt_pts: %ld, pkt_dts: %ld\n", test_inst->video_frame_count, test_inst->frame->pts, 
            //    test_inst->frame->pkt_pts, test_inst->frame->pkt_dts);

            if(test_inst->video_frame_count % 100 == 0) {
                test_inst->get_time_dec = av_gettime();
                if(test_inst->get_time_dec>test_inst->start_time_dec)
                test_inst->fps_dec = (double)test_inst->video_frame_count/((double)(test_inst->get_time_dec - test_inst->start_time_dec)/(1000*1000));
            }
            //av_usleep((rand()%200)*1000);
/*            printf("video_frame%s n:%d coded_n:%d\n",
                   cached ? "(cached)" : "",
                   test_inst->video_frame_count++, test_inst->frame->coded_picture_number);
*/
        }
    } else if (test_inst->pkt.stream_index == test_inst->audio_stream_idx) {
        /* decode audio frame */
        //av_packet_unref(&test_inst->pkt); 
        decoded = test_inst->pkt.size;
    }

    /* If we use frame reference counting, we own the data and need
     * to de-reference it when we don't use it anymore */
    if (*got_frame/* && test_inst->refcount*/) {
        int64_t in_pts, out_pts;
        struct timeval tv;
        in_pts = test_inst->pkt.pts;
        out_pts = test_inst->frame->pts;

        if(gettimeofday(&tv, NULL) == 0) {
            test_inst->output_pts = (int64_t)tv.tv_sec*1000 + tv.tv_usec/1000 - test_inst->start_msg_pts;
        }
        else {
            test_inst->output_pts = 0;
        }

        test_inst->input_pts = av_rescale_q(out_pts - test_inst->first_pkt_pts, test_inst->video_stream->time_base, pts_to_ms);
        test_inst->total_delay = test_inst->output_pts - test_inst->input_pts;
        test_inst->ave_delay = in_pts - out_pts;
        test_inst->ave_delay = av_rescale_q(test_inst->ave_delay, test_inst->video_stream->time_base, pts_to_ms);
            
        av_frame_unref(test_inst->frame);
    }

    return decoded;
}
static AVCodec *find_codec_or_die(const char *name, enum AVMediaType type, int encoder)
{
    const AVCodecDescriptor *desc;
    const char *codec_string = encoder ? "encoder" : "decoder";
    AVCodec *codec;

    codec = encoder ?
        avcodec_find_encoder_by_name(name) :
        avcodec_find_decoder_by_name(name);

    if (!codec && (desc = avcodec_descriptor_get_by_name(name))) {
        codec = encoder ? avcodec_find_encoder(desc->id) :
                          avcodec_find_decoder(desc->id);
        if (codec)
            av_log(NULL, AV_LOG_VERBOSE, "Matched %s '%s' for codec '%s'.\n",
                   codec_string, codec->name, desc->name);
    }

    if (!codec) {
        av_log(NULL, AV_LOG_FATAL, "Unknown %s '%s'\n", codec_string, name);
        exit(1);
    }
    if (codec->type != type) {
        av_log(NULL, AV_LOG_FATAL, "Invalid %s type '%s'\n", codec_string, name);
        exit(1);
    }
    return codec;
}
int pic_mode = 0;
static int open_codec_context(int *stream_idx,
                              AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx, enum AVMediaType type)
{
    int ret, stream_index;
    AVStream *st;
    AVCodec *dec = NULL;
    AVDictionary *opts = NULL;
    ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0);
    if (ret < 0) {
        fprintf(stderr, "Could not find %s stream in input file\n",
                av_get_media_type_string(type));
        return ret;
    } else {
        stream_index = ret;
        st = fmt_ctx->streams[stream_index];

        /* find decoder for the stream */
        if(codec_name_flag && type==AVMEDIA_TYPE_VIDEO) {
            dec = find_codec_or_die(codec_name   , AVMEDIA_TYPE_VIDEO   , 0);
        }
        else
            dec = avcodec_find_decoder(st->codecpar->codec_id);
        if (!dec) {
            fprintf(stderr, "Failed to find %s codec\n",
                    av_get_media_type_string(type));
            return AVERROR(EINVAL);
        }

        /* Allocate a codec context for the decoder */
        *dec_ctx = avcodec_alloc_context3(dec);
        if (!*dec_ctx) {
            fprintf(stderr, "Failed to allocate the %s codec context\n",
                    av_get_media_type_string(type));
            return AVERROR(ENOMEM);
        }

        /* Copy codec parameters from input stream to output codec context */
        if ((ret = avcodec_parameters_to_context(*dec_ctx, st->codecpar)) < 0) {
            fprintf(stderr, "Failed to copy %s codec parameters to decoder context\n",
                    av_get_media_type_string(type));
            return ret;
        }
        //av_dict_set(&opts, "extra_frame_buffer_num", "5", 18);
        //av_dict_set(&opts, "frame_delay", "1", 0);
            /* Init the decoders, with or without reference counting */
        if(pic_mode == 2)
                av_dict_set(&opts, "mode_bitstream", "2", 18);
        else if(pic_mode == 100)
            av_dict_set(&opts, "output_format", "100", 18);
        else if(pic_mode == 101)
            av_dict_set(&opts, "output_format", "101", 18);
//        av_dict_set(&opts, "extra_frame_buffer_num", "8", 18);
//        av_dict_set(&opts, "extra_data_flag", "8", 18);
        if ((ret = avcodec_open2(*dec_ctx, dec, &opts)) < 0) {
            fprintf(stderr, "Failed to open %s codec\n",
                    av_get_media_type_string(type));
            return ret;
        }
        *stream_idx = stream_index;
    }

    return 0;
}

static int get_format_from_sample_fmt(const char **fmt,
                                      enum AVSampleFormat sample_fmt)
{
    int i;
    struct sample_fmt_entry {
        enum AVSampleFormat sample_fmt; const char *fmt_be, *fmt_le;
    } sample_fmt_entries[] = {
        { AV_SAMPLE_FMT_U8,  "u8",    "u8"    },
        { AV_SAMPLE_FMT_S16, "s16be", "s16le" },
        { AV_SAMPLE_FMT_S32, "s32be", "s32le" },
        { AV_SAMPLE_FMT_FLT, "f32be", "f32le" },
        { AV_SAMPLE_FMT_DBL, "f64be", "f64le" },
    };
    *fmt = NULL;

    for (i = 0; i < FF_ARRAY_ELEMS(sample_fmt_entries); i++) {
        struct sample_fmt_entry *entry = &sample_fmt_entries[i];
        if (sample_fmt == entry->sample_fmt) {
            *fmt = AV_NE(entry->fmt_be, entry->fmt_le);
            return 0;
        }
    }

    fprintf(stderr,
            "sample format %s is not supported as output format\n",
            av_get_sample_fmt_name(sample_fmt));
    return -1;
}
#if 1


static int get_input_packet(MultiInstTest *f, AVPacket *pkt)
{
    /*
    if (f->rate_emu) {
        int i;
        for (i = 0; i < f->nb_streams; i++) {
            InputStream *ist = input_streams[f->ist_index + i];
            int64_t pts = av_rescale(ist->dts, 1000000, AV_TIME_BASE);
            int64_t now = av_gettime_relative() - ist->start;
            if (pts > now)
                return AVERROR(EAGAIN);
        }
    }

#if HAVE_THREADS
    if (inst_num > 1)
        return get_input_packet_mt(f, pkt);
#endif*/
    int ret = 0;
    ret = av_read_frame(f->fmt_ctx, pkt);
/*    if(ret == AVERROR_EOF || pkt->size == 0) {
        av_packet_unref(pkt);
        avformat_seek_file(f->fmt_ctx, -1, INT64_MIN, 0, INT64_MAX, AVSEEK_FLAG_ANY);
        av_log(f->fmt_ctx, AV_LOG_ERROR, "set bankword.......\n");
        ret = av_read_frame(f->fmt_ctx, pkt);
        av_log(f->fmt_ctx, AV_LOG_ERROR, "after set bankword.......ret: %d\n", ret);
    }
    */
    return ret;
}

#endif

static int AVInterruptCallBackFun(void *param) {
    MultiInstTest *test_inst = (MultiInstTest *)param;
    unsigned int time_temp = 0;
    time_temp = av_gettime() - test_inst->last_time_read_pkt;
    if(time_temp > 3*1000*1000) {
        return 1;
    }
    return 0;
}

static void *start_one_inst(void *arg)
{
    int ret = 0, got_frame;
    MultiInstTest *test_inst = (MultiInstTest *)arg;
    AVDictionary *dict = NULL;
    av_dict_set(&dict, "buffer_size", "1024000", 0);
    av_dict_set(&dict, "max_delay", "500000", 0);
    av_dict_set(&dict, "stimeout", "2000000", 0); 
    av_dict_set(&dict, "rtsp_transport", "tcp", 0);
    av_dict_set(&dict, "analyzeduration", "10", 0);
    av_dict_set(&dict, "probesize", "500", 0);
    //av_dict_set(&dict, "discardcorrupt", "0x0100", 0);
    //av_dict_set(&dict, "keep_rtsp_timestamp", "1", 0);
while(1) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	test_inst->start_msg_pts = (int64_t)tv.tv_sec*1000 + tv.tv_usec/1000;
    /* open input file, and allocate format context */
        if (avformat_open_input(&test_inst->fmt_ctx, test_inst->src_filename, NULL, &dict) < 0) {
          fprintf(stderr, "Could not open source file %s\n", test_inst->src_filename);
          usleep(1000*1000);
          continue;
          //exit(1);
        }

        /* retrieve stream information */
        if (avformat_find_stream_info(test_inst->fmt_ctx, NULL) < 0) {
          fprintf(stderr, "Could not find stream information\n");
          exit(1);
        }

        if (open_codec_context(&test_inst->video_stream_idx, &test_inst->video_dec_ctx, test_inst->fmt_ctx, AVMEDIA_TYPE_VIDEO) >= 0) {
          test_inst->video_stream = test_inst->fmt_ctx->streams[test_inst->video_stream_idx];
        /*
          test_inst->video_dst_file = fopen(test_inst->video_dst_filename, "wb");
          if (!test_inst->video_dst_file) {
              fprintf(stderr, "Could not open destination file %s\n", test_inst->video_dst_filename);
              ret = 1;
              goto end;
          }
        */
          /* allocate image where the decoded image will be put */
          test_inst->width = test_inst->video_dec_ctx->width;
          test_inst->height = test_inst->video_dec_ctx->height;
          if(codec_name_flag)
            test_inst->pix_fmt = AV_PIX_FMT_NV12;//test_inst->video_dec_ctx->pix_fmt;
          else
            test_inst->pix_fmt = test_inst->video_dec_ctx->pix_fmt;
            /*
          ret = av_image_alloc(test_inst->video_dst_data, test_inst->video_dst_linesize,
                               test_inst->width, test_inst->height, test_inst->pix_fmt, 1);
          if (ret < 0) {
              fprintf(stderr, "Could not allocate raw video buffer\n");
              //exit(1);
          }
          test_inst->video_dst_bufsize = ret;
          */
        }
      
        if (open_codec_context(&test_inst->audio_stream_idx, &test_inst->audio_dec_ctx, test_inst->fmt_ctx, AVMEDIA_TYPE_AUDIO) >= 0) {
          test_inst->audio_stream = test_inst->fmt_ctx->streams[test_inst->audio_stream_idx];  /*
          audio_dst_file = fopen(audio_dst_filename, "wb");
          if (!audio_dst_file) {
              fprintf(stderr, "Could not open destination file %s\n", audio_dst_filename);
              ret = 1;
              goto end;
          }
              */
        }
    
        /* dump input information to stderr */
        av_dump_format(test_inst->fmt_ctx, 0, test_inst->src_filename, 0);
    
        if (/*!audio_stream && */!test_inst->video_stream) {
          fprintf(stderr, "Could not find audio or video stream in the input, aborting\n");
          ret = 1;
          exit(1);
        }
    
      test_inst->frame = av_frame_alloc();
      if (!test_inst->frame) {
          fprintf(stderr, "Could not allocate frame\n");
          ret = AVERROR(ENOMEM);
          exit(1);
      }
    
      /* initialize packet, set data to NULL, let the demuxer fill it */
      av_init_packet(&test_inst->pkt);
      test_inst->pkt.data = NULL;
      test_inst->pkt.size = 0;
      test_inst->thread_queue_size = 30;
/*
    if ((ret = init_input_threads(arg)) < 0)
        exit(1);
        */
    test_inst->last_time_read_pkt = av_gettime();
    test_inst->fmt_ctx->interrupt_callback.callback = AVInterruptCallBackFun;
    test_inst->fmt_ctx->interrupt_callback.opaque   = test_inst;
    if (test_inst->video_stream)
      printf("Demuxing video from file '%s'\n", test_inst->src_filename);
/*  if (audio_stream)
      printf("Demuxing audio from file '%s' into '%s'\n", src_filename, audio_dst_filename);
*/
    /* read frames from the file */
    while (1) {
        if(test_inst->end_of!=0) {
            break;
        }
        ret = get_input_packet(test_inst, &test_inst->pkt);

        if (ret == AVERROR(EAGAIN)) {
            if((av_gettime() - test_inst->last_time_read_pkt) > 1000*1000*60) {
                break;
            }
            av_usleep(10000);
            continue;
        }
        else if(ret < 0)
            break;
        test_inst->last_time_read_pkt = av_gettime();
        AVPacket orig_pkt = test_inst->pkt;
        if((test_inst->first_pkt_flag == 0) && (test_inst->pkt.pts != AV_NOPTS_VALUE)) {
            test_inst->first_pkt_flag = 1;
            test_inst->first_pkt_pts = test_inst->pkt.pts;
            printf("first pkt pts: %d\n", test_inst->first_pkt_pts);
            printf("base time: %d, real time: %d\n", test_inst->fmt_ctx->start_time, test_inst->fmt_ctx->start_time_realtime);
        }
          do {
              ret = decode_packet(test_inst, &got_frame, 0);
              if (ret < 0) 
                  break;
              test_inst->pkt.data += ret;
              test_inst->pkt.size -= ret;
          } while (test_inst->pkt.size > 0);
          av_packet_unref(&orig_pkt);
    }

    /* flush cached frames */
    test_inst->pkt.data = NULL;
    test_inst->pkt.size = 0;
    do {
      decode_packet(test_inst, &got_frame, 1);
      //av_packet_unref(&(test_inst->pkt));
    } while (got_frame);
    //free_input_threads(test_inst->inst_idx);
    printf("Demuxing bitmain succeeded. inst index: %d\n", test_inst->inst_idx);

    if (test_inst->video_stream) {
      printf("Play the output video file with the command:\n"
             "ffplay -f rawvideo -pix_fmt %s -video_size %dx%d %s\n",
             av_get_pix_fmt_name(test_inst->pix_fmt), test_inst->width, test_inst->height,
             test_inst->video_dst_filename);
    }
/*
  if (audio_stream) {
      enum AVSampleFormat sfmt = audio_dec_ctx->sample_fmt;
      int n_channels = audio_dec_ctx->channels;
      const char *fmt;

      if (av_sample_fmt_is_planar(sfmt)) {
          const char *packed = av_get_sample_fmt_name(sfmt);
          printf("Warning: the sample format the decoder produced is planar "
                 "(%s). This example will output the first channel only.\n",
                 packed ? packed : "?");
          sfmt = av_get_packed_sample_fmt(sfmt);
          n_channels = 1;
      }

      if ((ret = get_format_from_sample_fmt(&fmt, sfmt)) < 0)
          goto end;

      printf("Play the output audio file with the command:\n"
             "ffplay -f %s -ac %d -ar %d %s\n",
             fmt, n_channels, audio_dec_ctx->sample_rate,
             audio_dst_filename);
  }
*/
end:
//  avcodec_close(test_inst->video_dec_ctx);
printf("close the decoder....\n");
  avcodec_free_context(&test_inst->video_dec_ctx);
  avcodec_free_context(&test_inst->audio_dec_ctx);
  avformat_close_input(&test_inst->fmt_ctx);
/*  if (test_inst->video_dst_file)
      fclose(test_inst->video_dst_file);
  if (test_inst->audio_dst_file)
      fclose(test_inst->audio_dst_file);
  */
 printf("close the frame....\n");
  av_frame_free(&test_inst->frame);
  //av_free(test_inst->video_dst_data[0]);
printf("check the end_of....\n");
  if(test_inst->end_of!=0) {
      printf(" the end_of and break.......\n");
      break;
    }
  //av_packet_free();
}
  return NULL;
}
#if HAVE_PTHREADS
pthread_t thread_id[MAX_INST_NUM];

#endif


/* read a key without blocking */
static int read_key(void)
{
    unsigned char ch;
#if HAVE_TERMIOS_H
    int n = 1;
    struct timeval tv;
    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    n = select(1, &rfds, NULL, NULL, &tv);
    if (n > 0) {
        n = read(0, &ch, 1);
        if (n == 1)
            return ch;

        return n;
    }
#elif HAVE_KBHIT
#    if HAVE_PEEKNAMEDPIPE
    static int is_pipe;
    static HANDLE input_handle;
    DWORD dw, nchars;
    if(!input_handle){
        input_handle = GetStdHandle(STD_INPUT_HANDLE);
        is_pipe = !GetConsoleMode(input_handle, &dw);
    }

    if (is_pipe) {
        /* When running under a GUI, you will end here. */
        if (!PeekNamedPipe(input_handle, NULL, 0, NULL, &nchars, NULL)) {
            // input pipe may have been closed by the program that ran ffmpeg
            return -1;
        }
        //Read it
        if(nchars != 0) {
            read(0, &ch, 1);
            return ch;
        }else{
            return -1;
        }
    }
#    endif
    if(kbhit())
        return(getch());
#endif
    return -1;
}
#include <semaphore.h>

#define VID_OPEN_SEM_NAME "/vid_open_global_sem"
#define CORE_SEM_NAME "/vpu_sem_core"
#define CORE_DISP_SEM_NEME "/vpu_disp_sem_core"

#ifdef __LOG4C__
#include "log4c.h"

static log4c_category_t *log_category = NULL;

static void custom_output(void* ptr, int level, const char* fmt,va_list vl)
{
    int a_priority = 0;
    switch(level) {
        case AV_LOG_QUIET:
            a_priority = LOG4C_PRIORITY_NOTSET;
            break;
        case AV_LOG_PANIC:
            a_priority = LOG4C_PRIORITY_NOTSET;
            break;
        case AV_LOG_FATAL:
            a_priority = LOG4C_PRIORITY_FATAL;
            break;
        case AV_LOG_ERROR:
            a_priority = LOG4C_PRIORITY_ERROR;
            break;
        case AV_LOG_WARNING:
            a_priority = LOG4C_PRIORITY_WARN;
            break;
        case AV_LOG_INFO:
            a_priority = LOG4C_PRIORITY_INFO;
            break;
        case AV_LOG_VERBOSE:
            a_priority = LOG4C_PRIORITY_NOTICE;
            break;
        case AV_LOG_DEBUG:
            a_priority = LOG4C_PRIORITY_DEBUG;
            break;
        case AV_LOG_TRACE:
            a_priority = LOG4C_PRIORITY_TRACE;
            break;
        default:
            a_priority = LOG4C_PRIORITY_NOTSET;
            break;
    }
    log4c_category_vlog(log_category, a_priority, fmt, vl);
}

#endif

int main (int argc, char **argv)
{
    int ret = 0, got_frame;
    int i=0;
    static int64_t last_time;
    int argc_offset = 0;

    if (argc > 2 && !strcmp(argv[1], "-c")) {
        strcpy(codec_name, argv[2]);
        codec_name_flag = 1;
        argc_offset = 2;
        argc -= 2;
    }
    else if(argc > 2 && !strcmp(argv[1], "-r")) {
        printf("reset all semaphore...\n");
        argc_offset = 1;
        argc -= 1;
        char sem_name[255];
        char disp_sem_name[255];
        int core_idx = 0;

        for(core_idx = 0; core_idx < 3; core_idx++) {
            sprintf(sem_name, "%s%d", CORE_SEM_NAME, core_idx);
            sprintf(disp_sem_name, "%s%d", CORE_DISP_SEM_NEME, core_idx);
            printf("sem_name unlink : %s\n", sem_name);

            sem_unlink(sem_name);

            sem_unlink(disp_sem_name);
        }
        sem_unlink(VID_OPEN_SEM_NAME);
    }
    else if(argc > 2 && !strcmp(argv[1], "-p")) {
        pic_mode = 2;
        argc_offset = 1;
        argc -= 1;
    }
    else if(argc > 2 && !strcmp(argv[1], "-co")) {
        pic_mode = 101;
        argc_offset = 1;
        argc -= 1;
    }
    else if(argc > 2 && !strcmp(argv[1], "-ti")) {
        pic_mode = 100;
        argc_offset = 1;
        argc -= 1;
    }
    else if(argc > 2 && !strcmp(argv[1], "-inst_num")) {
        inst_num = atoi(argv[2]);
        printf("instance number : %d\n", inst_num);
        argc_offset = 2;
        argc -= 2;
    }

    if (argc < 2 || argc > MAX_INST_NUM + 1) {
        fprintf(stderr, "usage: %s input_file [inst_num]\n"
                "API example program to show how to read frames from an input file.\n"
                "This program reads frames from a file, decodes them, and writes decoded\n"
                "video frames to a rawvideo file named video_output_file, and decoded\n"
                "audio frames to a rawaudio file named audio_output_file.\n\n"
                "If the -refcount option is specified, the program use the\n"
                "reference counting frame system which allows keeping a copy of\n"
                "the data for longer than one decode call.\n"
                "\n", argv[0]);
        exit(1);
    }
/*    if (argc == 5 && !strcmp(argv[1], "-refcount")) {
        refcount = 1;
        argv++;
    }*/
    if(inst_num == 0)
        inst_num = argc-1;

    if(inst_num > MAX_INST_NUM) {
        printf(".....exit, %d\n", inst_num);
        exit(1);
    }

    if(argc < 2) {
        printf("please set input file name.....exit, %d\n");
        exit(1);
    }
#ifdef __LOG4C__
    log4c_init();
    log_category = log4c_category_get("ffmpeg");
    /*
    log4c_appender_t* myappender;
    myappender = log4c_appender_get("./mylog615.log");
    log4c_appender_set_type(myappender,log4c_appender_type_get("stream2"));
    log4c_category_set_appender(log_category, myappender);*/
    av_log_set_callback(custom_output);
    av_log_set_level(AV_LOG_INFO);
#endif
    for(i=1; i<argc; i++) {
        inst[i-1].src_filename = argv[i + argc_offset];
        inst[i-1].end_of = 0;
        inst[i-1].first_frame_flag = 0;
        inst[i-1].start_time_dec = 0;
        inst[i-1].get_time_dec = 0;
        inst[i-1].fps_dec = 0;
        inst[i-1].inst_idx = i-1;
    }
    for(;i<inst_num+1; i++) {
        inst[i-1].src_filename = argv[1 + argc_offset];
        inst[i-1].end_of = 0;
        inst[i-1].first_frame_flag = 0;
        inst[i-1].start_time_dec = 0;
        inst[i-1].get_time_dec = 0;
        inst[i-1].fps_dec = 0;
        inst[i-1].inst_idx = i-1;
    }
    av_log_set_level(AV_LOG_ERROR);
    //open_all_file();
    /*
#if HAVE_THREADS
    if ((ret = init_input_threads()) < 0)
        exit(1);
#endif
*/
//    video_dst_filename = argv[2];
//    audio_dst_filename = argv[3];
#if HAVE_PTHREADS
    for(i=0; i<inst_num; i++)
    {
        pthread_create(&(thread_id[i]), NULL, start_one_inst, &(inst[i]));
//        av_usleep(5*1000*1000);
    }
    int key;

    while(1)
    {
        for(i=0; i<inst_num; i++) {
            printf("[%d], [%10d], [%2.2f], {decoder delay:%d ms}, {client pts: %d ms}, {server pts: %d ms}, {delay: %d ms}\n", i, inst[i].video_frame_count, inst[i].fps_dec, inst[i].ave_delay, inst[i].input_pts, inst[i].output_pts, inst[i].total_delay);
            inst[i].ave_delay = 0;
        }
        printf("\r");
        fflush(stdout);
        
        key = read_key();
        if (key == 'q') {            
            for(i=0; i<inst_num; i++) {
                inst[i].end_of = 1;
            }
            for(i=0; i<inst_num; i++)
            {
                pthread_join(thread_id[i], NULL);
            }

            break;
        }
        
        av_usleep(5*1000*1000);
    }

#endif
#ifdef __LOG4C__
    log4c_fini();
#endif
    return ret < 0;
}
