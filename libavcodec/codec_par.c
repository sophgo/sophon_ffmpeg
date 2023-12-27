/*
 * AVCodecParameters functions for libavcodec
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
 * AVCodecParameters functions for libavcodec.
 */

#include <string.h>
#include "libavutil/mem.h"
#include "avcodec.h"
#include "codec_par.h"
#include "config_components.h"

static void codec_parameters_reset(AVCodecParameters *par)
{
    av_freep(&par->extradata);
    av_channel_layout_uninit(&par->ch_layout);

    memset(par, 0, sizeof(*par));

    par->codec_type          = AVMEDIA_TYPE_UNKNOWN;
    par->codec_id            = AV_CODEC_ID_NONE;
    par->format              = -1;
    par->ch_layout.order     = AV_CHANNEL_ORDER_UNSPEC;
    par->field_order         = AV_FIELD_UNKNOWN;
    par->color_range         = AVCOL_RANGE_UNSPECIFIED;
    par->color_primaries     = AVCOL_PRI_UNSPECIFIED;
    par->color_trc           = AVCOL_TRC_UNSPECIFIED;
    par->color_space         = AVCOL_SPC_UNSPECIFIED;
    par->chroma_location     = AVCHROMA_LOC_UNSPECIFIED;
    par->sample_aspect_ratio = (AVRational){ 0, 1 };
    par->profile             = FF_PROFILE_UNKNOWN;
    par->level               = FF_LEVEL_UNKNOWN;
}

AVCodecParameters *avcodec_parameters_alloc(void)
{
    AVCodecParameters *par = av_mallocz(sizeof(*par));

    if (!par)
        return NULL;
    codec_parameters_reset(par);
    return par;
}

void avcodec_parameters_free(AVCodecParameters **ppar)
{
    AVCodecParameters *par = *ppar;

    if (!par)
        return;
    codec_parameters_reset(par);

    av_freep(ppar);
}

int avcodec_parameters_copy(AVCodecParameters *dst, const AVCodecParameters *src)
{
    int ret;

    codec_parameters_reset(dst);
    memcpy(dst, src, sizeof(*dst));

    dst->ch_layout      = (AVChannelLayout){0};
    dst->extradata      = NULL;
    dst->extradata_size = 0;
    if (src->extradata) {
        dst->extradata = av_mallocz(src->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!dst->extradata)
            return AVERROR(ENOMEM);
        memcpy(dst->extradata, src->extradata, src->extradata_size);
        dst->extradata_size = src->extradata_size;
    }

    ret = av_channel_layout_copy(&dst->ch_layout, &src->ch_layout);
    if (ret < 0)
        return ret;

    return 0;
}

int avcodec_parameters_from_context(AVCodecParameters *par,
                                    const AVCodecContext *codec)
{
    int ret;

    codec_parameters_reset(par);

    par->codec_type = codec->codec_type;
    par->codec_id   = codec->codec_id;
    par->codec_tag  = codec->codec_tag;

    par->bit_rate              = codec->bit_rate;
    par->bits_per_coded_sample = codec->bits_per_coded_sample;
    par->bits_per_raw_sample   = codec->bits_per_raw_sample;
    par->profile               = codec->profile;
    par->level                 = codec->level;

    switch (par->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
        par->format              = codec->pix_fmt;
        par->width               = codec->width;
        par->height              = codec->height;
        par->field_order         = codec->field_order;
        par->color_range         = codec->color_range;
        par->color_primaries     = codec->color_primaries;
        par->color_trc           = codec->color_trc;
        par->color_space         = codec->colorspace;
        par->chroma_location     = codec->chroma_sample_location;
        par->sample_aspect_ratio = codec->sample_aspect_ratio;
        par->video_delay         = codec->has_b_frames;
        break;
    case AVMEDIA_TYPE_AUDIO:
        par->format           = codec->sample_fmt;
#if FF_API_OLD_CHANNEL_LAYOUT
FF_DISABLE_DEPRECATION_WARNINGS
        // if the old/new fields are set inconsistently, prefer the old ones
        if ((codec->channels && codec->channels != codec->ch_layout.nb_channels) ||
            (codec->channel_layout && (codec->ch_layout.order != AV_CHANNEL_ORDER_NATIVE ||
                                       codec->ch_layout.u.mask != codec->channel_layout))) {
            if (codec->channel_layout)
                av_channel_layout_from_mask(&par->ch_layout, codec->channel_layout);
            else {
                par->ch_layout.order       = AV_CHANNEL_ORDER_UNSPEC;
                par->ch_layout.nb_channels = codec->channels;
            }
FF_ENABLE_DEPRECATION_WARNINGS
        } else {
#endif
        ret = av_channel_layout_copy(&par->ch_layout, &codec->ch_layout);
        if (ret < 0)
            return ret;
#if FF_API_OLD_CHANNEL_LAYOUT
FF_DISABLE_DEPRECATION_WARNINGS
        }
        par->channel_layout  = par->ch_layout.order == AV_CHANNEL_ORDER_NATIVE ?
                               par->ch_layout.u.mask : 0;
        par->channels        = par->ch_layout.nb_channels;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
        par->sample_rate      = codec->sample_rate;
        par->block_align      = codec->block_align;
        par->frame_size       = codec->frame_size;
        par->initial_padding  = codec->initial_padding;
        par->trailing_padding = codec->trailing_padding;
        par->seek_preroll     = codec->seek_preroll;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        par->width  = codec->width;
        par->height = codec->height;
        break;
    }

    if (codec->extradata) {
        par->extradata = av_mallocz(codec->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!par->extradata)
            return AVERROR(ENOMEM);
        memcpy(par->extradata, codec->extradata, codec->extradata_size);
        par->extradata_size = codec->extradata_size;
    }

    return 0;
}

//in bm decoder we set the default output format to nv12
static const char* bm_find_decoder_name(int dec_id)
{
    switch (dec_id) {
#if CONFIG_JPEG_BM_DECODER
        case AV_CODEC_ID_MJPEG:     return "jpeg_bm";
#endif
#if CONFIG_H264_BM_DECODER
        case AV_CODEC_ID_H264:      return "h264_bm";
#endif
#if CONFIG_HEVC_BM_DECODER
        case AV_CODEC_ID_HEVC:      return "hevc_bm";
#endif
#if CONFIG_MPEG1_BM_DECODER
        case AV_CODEC_ID_MPEG1VIDEO:return "mpeg1_bm";
#endif
#if CONFIG_MPEG2_BM_DECODER
        case AV_CODEC_ID_MPEG2VIDEO:return "mpeg2_bm";
#endif
#if CONFIG_MPEG4_BM_DECODER
        case AV_CODEC_ID_MPEG4:     return "mpeg4_bm";
#endif
#if CONFIG_MPEG4V3_BM_DECODER
        case AV_CODEC_ID_MSMPEG4V3: return "mpeg4v3_bm";
#endif
#if CONFIG_FLV1_BM_DECODER
        case AV_CODEC_ID_FLV1:      return "flv1_bm";
#endif

#if CONFIG_H263_BM_DECODER
        case AV_CODEC_ID_H263:      return "h263_bm";
#endif

#if CONFIG_CAVS_BM_DECODER
        case AV_CODEC_ID_CAVS:      return "cavs_bm";
#endif
#if CONFIG_AVS_BM_DECODER
        case AV_CODEC_ID_AVS:       return "avs_bm";
#endif

#if CONFIG_VP3_BM_DECODER
        case AV_CODEC_ID_VP3:       return "vp3_bm";
#endif
#if CONFIG_VP8_BM_DECODER
    case AV_CODEC_ID_VP8:       return "vp8_bm";
#endif
#if CONFIG_VC1_BM_DECODER
        case AV_CODEC_ID_VC1:       return "vc1_bm";
#endif
#if CONFIG_WMV1_BM_DECODER
        case AV_CODEC_ID_WMV1:      return "wmv1_bm";
#endif
#if CONFIG_WMV2_BM_DECODER
        case AV_CODEC_ID_WMV2:      return "wmv2_bm";
#endif
#if CONFIG_WMV3_BM_DECODER
        case AV_CODEC_ID_WMV3:      return "wmv3_bm";
#endif
        default:                    return NULL;
    }

    return NULL;
}


int avcodec_parameters_to_context(AVCodecContext *codec,
                                  const AVCodecParameters *par)
{
    int ret;

    codec->codec_type = par->codec_type;
    codec->codec_id   = par->codec_id;
    codec->codec_tag  = par->codec_tag;

    codec->bit_rate              = par->bit_rate;
    codec->bits_per_coded_sample = par->bits_per_coded_sample;
    codec->bits_per_raw_sample   = par->bits_per_raw_sample;
    codec->profile               = par->profile;
    codec->level                 = par->level;

    switch (par->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
        if(bm_find_decoder_name(par->codec_id) && par->format == AV_PIX_FMT_YUV420P)
            codec->pix_fmt = AV_PIX_FMT_NV12;
        else
            codec->pix_fmt                = par->format;
        codec->width                  = par->width;
        codec->height                 = par->height;
        codec->field_order            = par->field_order;
        codec->color_range            = par->color_range;
        codec->color_primaries        = par->color_primaries;
        codec->color_trc              = par->color_trc;
        codec->colorspace             = par->color_space;
        codec->chroma_sample_location = par->chroma_location;
        codec->sample_aspect_ratio    = par->sample_aspect_ratio;
        codec->has_b_frames           = par->video_delay;
        break;
    case AVMEDIA_TYPE_AUDIO:
        codec->sample_fmt       = par->format;
#if FF_API_OLD_CHANNEL_LAYOUT
FF_DISABLE_DEPRECATION_WARNINGS
        // if the old/new fields are set inconsistently, prefer the old ones
        if ((par->channels && par->channels != par->ch_layout.nb_channels) ||
            (par->channel_layout && (par->ch_layout.order != AV_CHANNEL_ORDER_NATIVE ||
                                     par->ch_layout.u.mask != par->channel_layout))) {
            if (par->channel_layout)
                av_channel_layout_from_mask(&codec->ch_layout, par->channel_layout);
            else {
                codec->ch_layout.order       = AV_CHANNEL_ORDER_UNSPEC;
                codec->ch_layout.nb_channels = par->channels;
            }
FF_ENABLE_DEPRECATION_WARNINGS
        } else {
#endif
        ret = av_channel_layout_copy(&codec->ch_layout, &par->ch_layout);
        if (ret < 0)
            return ret;
#if FF_API_OLD_CHANNEL_LAYOUT
FF_DISABLE_DEPRECATION_WARNINGS
        }
        codec->channel_layout = codec->ch_layout.order == AV_CHANNEL_ORDER_NATIVE ?
                                codec->ch_layout.u.mask : 0;
        codec->channels       = codec->ch_layout.nb_channels;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
        codec->sample_rate      = par->sample_rate;
        codec->block_align      = par->block_align;
        codec->frame_size       = par->frame_size;
        codec->delay            =
        codec->initial_padding  = par->initial_padding;
        codec->trailing_padding = par->trailing_padding;
        codec->seek_preroll     = par->seek_preroll;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        codec->width  = par->width;
        codec->height = par->height;
        break;
    }

    if (par->extradata) {
        av_freep(&codec->extradata);
        codec->extradata = av_mallocz(par->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!codec->extradata)
            return AVERROR(ENOMEM);
        memcpy(codec->extradata, par->extradata, par->extradata_size);
        codec->extradata_size = par->extradata_size;
    }

    return 0;
}
