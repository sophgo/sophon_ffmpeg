/*
 * gb28181 definitions
 *
 */

#include "gb28181.h"
#include <gb28181_sip_cinterface.h>

#define OFFSET(x) offsetof(RegistrationInfo, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM
#define ENC AV_OPT_FLAG_ENCODING_PARAM


const AVOption ff_gb28181pb_options[] = {
    { "sip_min_port",   "set minimum local sip port",          OFFSET(sip_port_min), AV_OPT_TYPE_INT, {.i64 = GB28181_SIP_PORT_MIN}, 0, 65535, DEC|ENC },
    { "sip_max_port",   "set maximum local sip port",          OFFSET(sip_port_max), AV_OPT_TYPE_INT, {.i64 = GB28181_SIP_PORT_MAX}, 0, 65535, DEC|ENC },
    { "rtp_min_port",   "set minimum local rtp port",          OFFSET(rtp_port_min), AV_OPT_TYPE_INT, {.i64 = GB28181_RTP_PORT_MIN}, 0, 65535, DEC|ENC },
    { "rtp_max_port",   "set maximum local rtp port",          OFFSET(rtp_port_max), AV_OPT_TYPE_INT, {.i64 = GB28181_RTP_PORT_MAX}, 0, 65535, DEC|ENC },
    { "buffer_size","Send/Receive buffer size (in bytes)", OFFSET(buffer_size),  AV_OPT_TYPE_INT, { .i64 = -1 },    -1, INT_MAX, DEC|ENC },
    { "gb28181_transport_sip", "set gb28181 sip transport protocols", OFFSET(transport_sip_mask), AV_OPT_TYPE_FLAGS, {.i64 = GB28181_TRANSPORT_UDP}, INT_MIN, INT_MAX, DEC|ENC, "gb28181_transport_sip" },
    { "udp", "UDP", 0, AV_OPT_TYPE_CONST, {.i64 = GB28181_TRANSPORT_UDP}, 0, 0, DEC|ENC, "gb28181_transport_sip" },
    { "tcp", "TCP", 0, AV_OPT_TYPE_CONST, {.i64 = GB28181_TRANSPORT_TCP}, 0, 0, DEC|ENC, "gb28181_transport_sip" },
    { "gb28181_transport_rtp", "set gb28181 rtp transport protocols", OFFSET(transport_rtp_mask), AV_OPT_TYPE_FLAGS, {.i64 = GB28181_TRANSPORT_UDP}, INT_MIN, INT_MAX, DEC|ENC, "gb28181_transport_rtp" },
    { "udp", "UDP", 0, AV_OPT_TYPE_CONST, {.i64 = GB28181_TRANSPORT_UDP}, 0, 0, DEC|ENC, "gb28181_transport_rtp" },
    { "tcp", "TCP", 0, AV_OPT_TYPE_CONST, {.i64 = GB28181_TRANSPORT_TCP}, 0, 0, DEC|ENC, "gb28181_transport_rtp" },
    { NULL },
};



static int gb28181pb_probe(AVProbeData *p) {
    if (av_strstart(p->filename, "gb28181_playback:",NULL)) {
      return AVPROBE_SCORE_MAX;
    }
    return 0;
}

static int gb28181pb_read_header(AVFormatContext *s) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    char *probe_buf = NULL;
    unsigned int probe_len = 0;
    unsigned int packet_nums = 0;
    int ret;

    printf("read_header pb url=%s\n",s->url);
    rt->register_success = false;
    rt->invite_answer    = false;
    if (s->max_delay < 0) /* Not set by the caller */
        s->max_delay = GB28181_DEFAULT_REORDERING_DELAY;

    //Parsing url parameters
    ret = ff_gb28181_parm_parse(s);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR,"sip gb28181_playback parm is err.\n");
        printf("sip parm parse failed.\n");
        return ret;
    }


    RegistrationInfo *rt_info = (RegistrationInfo *)s->priv_data;
    if(rt->transport_sip_mask == GB28181_TRANSPORT_UDP)
        init_sip_recv(rt_info,&(rt_info->sip_handle),1);
    else
        init_sip_recv(rt_info,&(rt_info->sip_handle),0);


    send_register(&(rt_info->sip_handle));
    int idx=0;
    for(idx=0;idx<SIP_MSG_TIMEOUT;idx++){
        if (get_register_status(&(rt_info->sip_handle))) {
            break;
        }
        usleep(1000);
    }
    if(idx >= (SIP_MSG_TIMEOUT -1)){
        return -1;
    }

    if(rt->transport_rtp_mask == GB28181_TRANSPORT_UDP)
        send_invite(&(rt_info->sip_handle),1,rt->proto);
    else
        send_invite(&(rt_info->sip_handle),0,rt->proto);

    for(idx=0;idx<SIP_MSG_TIMEOUT;idx++){
        if (get_ack_status(&(rt_info->sip_handle))) {
            break;
        }
        usleep(1000);
    }
    if(idx >= (SIP_MSG_TIMEOUT -1))
        return -1;

    ret = ff_gb28181_media_setup_rtp_channel(s);
    if(ret < 0) {
        av_log(s, AV_LOG_ERROR,"sip media setup failed.\n");
        goto fail;
    }

    ret = ff_gb28181_media_create_avstream(s);
    if(ret < 0) {
        av_log(s, AV_LOG_ERROR,"sip media create failed.\n");
        goto fail;
    }

    ret = ff_gb28181_media_open_transport_ctx(s);
    if(ret < 0) {
        av_log(s, AV_LOG_ERROR,"sip media rtp parse failed.\n");
        goto fail;
    }

    probe_buf = (char*)malloc(1024*1024*10);
    probe_len = 0;
    packet_nums = 0;
    while (true) {
        AVInputFormat *fmt;
        AVPacketList *pict_list;
        AVProbeData pd = {"gb28181_stream"};
        int score = 25;

        pict_list = av_mallocz(sizeof(AVPacketList));
        if (!pict_list) {
            ret = AVERROR(ENOMEM);
            av_log(s, AV_LOG_ERROR,"sip media malloc(%ld) failed.\n", sizeof(AVPacketList));
            goto fail;
        }

        ret = ff_gb28181_read_packet(s,&(pict_list->pkt));
        if(ret == AVERROR(ETIMEDOUT)) {
            av_log(s, AV_LOG_ERROR,"sip media probe read packet timeout.\n");
            av_freep(pict_list);
            goto fail;
        }
        packet_nums++;
        if((pict_list->pkt.size <= 0)&&(packet_nums < 400)) {
            av_freep(pict_list);
            printf("read packet failed.  packet_nums=%d\n",packet_nums);
            continue;
        }

        memcpy(probe_buf+probe_len, pict_list->pkt.data, pict_list->pkt.size);
        probe_len += pict_list->pkt.size;
        if (rt->packet_buffer == NULL) {
            rt->packet_buffer = pict_list;
        } else {
            rt->packet_buffer_end->next = pict_list;
        }
        rt->packet_buffer_end = pict_list;

        if((packet_nums < 200) && (s->streams[0]->codecpar->codec_id == AV_CODEC_ID_NONE)) {
            av_freep(pict_list);
            continue;
        }

        probe_buf[probe_len] = '\0';
        pd.buf_size = probe_len;
        pd.buf = probe_buf;

        fmt = av_probe_input_format2(&pd, 1, &score);
        if(fmt != NULL) {
            s->streams[0]->codecpar->codec_id = fmt->raw_codec_id;
            printf("codec_id=%d\n",s->streams[0]->codecpar->codec_id);
        } else {
            printf("psm codec_id=%d\n",s->streams[0]->codecpar->codec_id);
        }
        av_freep(pict_list);
        break;
    }

    if (probe_buf != NULL) {
        free(probe_buf);
        probe_buf = NULL;
    }

    if(ret < 0){
        goto fail;
    }
    printf("read_header success deviceid=%s\n",rt->deviceid);

    return ret;
fail:
    ff_gb28181_media_close(s);
    send_bye(&(rt_info->sip_handle));
    close_gbclient(&rt->sip_handle);
    printf("read_header failed(%d) deviceid=%s\n",ret, rt->deviceid);
    return ret;
}

static int gb28181pb_read_packet(AVFormatContext *s, AVPacket *pkt) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    if (rt->packet_buffer != NULL) {
        AVPacketList *pktl;

        av_packet_ref(pkt,&(rt->packet_buffer->pkt));
        pktl = rt->packet_buffer;
        rt->packet_buffer = rt->packet_buffer->next;
        if(rt->packet_buffer == NULL)
            rt->packet_buffer_end = NULL;
        av_packet_unref(&(pktl->pkt));
        av_freep(pktl);
        return 0;
    }
    if(rt->bFileEnd) {
        //return eof.
        return AVERROR_EOF;
    }
    return ff_gb28181_read_packet(s,pkt);
}

static int gb28181pb_read_close(AVFormatContext *s) {
    RegistrationInfo *rt;
    if(s == NULL) {
        return 0;
    }

    rt = (RegistrationInfo *)s->priv_data;
    printf("read_close url=%s\n",s->url);
    ff_gb28181_media_close(s);

    send_bye(&(rt->sip_handle));
    return 0;
}

static int gb28181pb_read_play(AVFormatContext *s) {
    //ff_gb28181_sip_info_playback(s,"PLAY");
    RegistrationInfo *rt_info = (RegistrationInfo *)s->priv_data;
    send_play(&(rt_info));
    return 0;
}

static int gb28181pb_read_seek(AVFormatContext *s, int stream_index, int64_t timestamp, int flags)
{
    return 0;
}
static int gb28181pb_read_pause(AVFormatContext *s)
{
    //ff_gb28181_sip_info_playback(s,"PAUSE");
    RegistrationInfo *rt_info = (RegistrationInfo *)s->priv_data;
    send_pause(&(rt_info));
    return 0;
}

static const AVClass gb28181pb_demuxer_class = {
    .class_name     = "GB28181 demuxer",
    .item_name      = av_default_item_name,
    .option         = ff_gb28181pb_options,
    .version        = LIBAVUTIL_VERSION_INT,
};

AVInputFormat ff_gb28181pb_demuxer = {
    .name           = "gb28181pb",
    .long_name      = NULL_IF_CONFIG_SMALL("gb28181pb input"),
    .priv_data_size = sizeof(RegistrationInfo),
    .read_probe     = gb28181pb_probe,
    .read_header    = gb28181pb_read_header,
    .read_packet    = gb28181pb_read_packet,
    .read_close     = gb28181pb_read_close,
    .read_play      = gb28181pb_read_play,
    .read_seek      = gb28181pb_read_seek,
    .read_pause     = gb28181pb_read_pause,
    .flags          = AVFMT_NOFILE,
    .priv_class     = &gb28181pb_demuxer_class,
};


