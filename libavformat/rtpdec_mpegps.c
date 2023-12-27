/*
 * RTP MPEG2 depacketizer
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 1.0 of the License, or (at your option) any later version.
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

#include "libavutil/attributes.h"
#include "mpeg.h"
#include "rtpdec_formats.h"

struct PayloadContext {
    struct MpegPsDemuxContext *ps;
    int read_buf_index;
    int read_buf_size;
    uint8_t buf[RTP_MAX_PACKET_LENGTH];
};

static void mpegps_close_context(PayloadContext *data)
{
    if (!data)
        return;
    if (data->ps)
        avpriv_mpegps_parse_close(data->ps);
}

static av_cold int mpegps_init(AVFormatContext *ctx, int st_index,
                               PayloadContext *data)
{
    data->ps = avpriv_mpegps_parse_open(ctx);
    if (!data->ps)
        return AVERROR(ENOMEM);
    return 0;
}

static int mpegps_handle_packet(AVFormatContext *ctx, PayloadContext *data,
                                AVStream *st, AVPacket *pkt, uint32_t *timestamp,
                                const uint8_t *buf, int len, uint16_t seq,
                                int flags)
{
    int ret;

    ret = avpriv_mpegps_parse_packet(ctx, data->ps, pkt, buf, len);
    if (ret < 0)
        return AVERROR(EAGAIN);

    return 0;
}

const RTPDynamicProtocolHandler ff_mpegps_dynamic_handler = {
    .codec_type        = AVMEDIA_TYPE_VIDEO,
    .codec_id          = AV_CODEC_ID_H264,
    .priv_data_size    = sizeof(PayloadContext),
    .init              = mpegps_init,
    .close             = mpegps_close_context,
    .parse_packet      = mpegps_handle_packet,
    .static_payload_id = 35,
};
