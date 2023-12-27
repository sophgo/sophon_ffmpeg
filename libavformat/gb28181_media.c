/*
 * gb28181 definitions
 *
 */

#include "gb28181.h"
#include "libavutil/intreadwrite.h"

static AVDictionary *map_to_opts(RegistrationInfo *rt) {
    AVDictionary *opts = NULL;
    char buf[256] = {0};

    snprintf(buf, sizeof(buf), "%d", rt->buffer_size);
    av_dict_set(&opts, "buffer_size", buf, 0);

    return opts;
}

int ff_gb28181_media_setup_rtp_channel(AVFormatContext *s)
{
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    char buf[256];
    int err;
    char options[1024] = {0};

    if(rt->transport_rtp_mask == GB28181_TRANSPORT_UDP) {
        //udp
        AVDictionary *opts = map_to_opts(rt);
        ff_url_join(buf, sizeof(buf), "rtp", NULL, rt->media_ip,/*rt->localip,*/ -1,
                    "?localport=%d", rt->localportrtp);
        err = ffurl_open_whitelist(&rt->rtp_handle, buf, AVIO_FLAG_READ_WRITE,
                                   &s->interrupt_callback, &opts,
                                   s->protocol_whitelist, s->protocol_blacklist, NULL);
        if(err < 0){
            return err;
        }
        ff_url_join(buf, sizeof(buf), "udp", NULL, rt->media_ip,rt->media_port,"%s",options);
        ff_rtp_set_remote_url(rt->rtp_handle, buf);

        av_dict_free(&opts);
        if(!err) {
            return rt->localportrtp;
        }
    } else {
        ////tcp
        //ff_url_join(buf, sizeof(buf), "tcp", NULL,
        //            rt->localip,rt->localportrtp,
        //            "?listen=%d", 2);
        ff_url_join(buf, sizeof(buf), "tcp", NULL,
                    rt->media_ip,rt->media_port,
                    //"10.33.38.20",5154,
                    "?timeout=%d", 0);
        err = ffurl_open_whitelist(&rt->rtp_handle, buf, AVIO_FLAG_READ_WRITE,
                                   &s->interrupt_callback, NULL, s->protocol_whitelist,
                                   s->protocol_blacklist, NULL);
        if(err >= 0) {
            return rt->media_port;
        }
    }
    return err;
}




int ff_gb28181_media_create_avstream(AVFormatContext *s) {
    //RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    AVStream *st = NULL;
    FFStream *sti;
    st = avformat_new_stream(s, NULL);
    if (!st)
        return -1;
    st->id = s->nb_streams - 1;
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_H264;
    sti = ffstream(st);
    sti->need_parsing=AVSTREAM_PARSE_FULL;
    return 0;
}

int ff_gb28181_media_open_transport_ctx(AVFormatContext *s) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    AVStream *st = NULL;
    AVCodecParameters *par = NULL;
    /* open the RTP context */
    if (rt->stream_index >= 0)
        st = s->streams[rt->stream_index];
    if (!st)
        s->ctx_flags |= AVFMTCTX_NOHEADER;

    rt->transport_priv = ff_rtp_parse_open(s, st,
            GB28181_SDP_PAYLOAD_TYPE, RTP_REORDER_QUEUE_DEFAULT_SIZE);

    if (!rt->transport_priv) {
         return AVERROR(ENOMEM);
    } else if (CONFIG_RTPDEC && s->iformat) {
        rt->dynamic_handler = ff_rtp_handler_find_by_id(35, AVMEDIA_TYPE_VIDEO);

        par = st ? st->codecpar : NULL;
        if (!rt->dynamic_handler)
            return -1;
        if (par) {
            par->codec_id          = AV_CODEC_ID_H264;
        }

        if (rt->dynamic_handler->priv_data_size) {
            rt->dynamic_protocol_context = av_mallocz(rt->dynamic_handler->priv_data_size);
            if (!rt->dynamic_protocol_context)
                rt->dynamic_handler = NULL;
        }

        if (rt->dynamic_handler && rt->dynamic_handler->init) {
            int ret = rt->dynamic_handler->init(s, st ? st->index : -1,
                                                     rt->dynamic_protocol_context);
            if (ret < 0) {
                if (rt->dynamic_protocol_context) {
                    if (rt->dynamic_handler->close)
                        rt->dynamic_handler->close(rt->dynamic_protocol_context);
                    av_free(rt->dynamic_protocol_context);
                }
                rt->dynamic_protocol_context = NULL;
                rt->dynamic_handler = NULL;
            }
        }

        //RTPDemuxContext *rtpctx = rt->transport_priv;
        if (rt->dynamic_handler) {
            ff_rtp_parse_set_dynamic_protocol(rt->transport_priv,
                                              rt->dynamic_protocol_context,
                                              rt->dynamic_handler);
        }
    }

    return 0;
}


int ff_gb28181_rtp_read_packet(AVFormatContext *s,uint8_t *buf, int buf_size,int64_t wait_end) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    int ret,n;
    struct pollfd *p = rt->p;
    int timeout_cnt = 0;
    int *fds = NULL, fdsnum, fdsidx;

    if (!p) {
        p = rt->p = av_malloc_array(2, sizeof(struct pollfd));
        if (!p)
            return AVERROR(ENOMEM);

        if ((rt->rtp_handle) && (rt->transport_rtp_mask == GB28181_TRANSPORT_UDP)) {
            if (ret = ffurl_get_multi_file_handle(rt->rtp_handle,
                                                  &fds, &fdsnum)) {
                av_log(s, AV_LOG_ERROR, "Unable to recover rtp ports\n");
                return ret;
            }
            if (fdsnum != 2) {
                av_log(s, AV_LOG_ERROR,
                       "Number of fds %d not supported\n", fdsnum);
                return AVERROR_INVALIDDATA;
            }
            for (fdsidx = 0; fdsidx < fdsnum; fdsidx++) {
                p[rt->max_p].fd       = fds[fdsidx];
                p[rt->max_p++].events = POLLIN;
            }
            av_freep(&fds);
        } else {
            if (ret = ffurl_get_multi_file_handle(rt->rtp_handle,
                                                  &fds, &fdsnum)) {
                av_log(s, AV_LOG_ERROR, "Unable to recover rtp ports\n");
                return ret;
            }
            if (fdsnum != 1) {
                av_log(s, AV_LOG_ERROR,
                       "Number of fds %d not supported\n", fdsnum);
                return AVERROR_INVALIDDATA;
            }
            for (fdsidx = 0; fdsidx < fdsnum; fdsidx++) {
                p[rt->max_p].fd       = fds[fdsidx];
                p[rt->max_p++].events = POLLIN;
            }
            av_freep(&fds);
        }
    }

    for (;;) {
        if (ff_check_interrupt(&s->interrupt_callback))
            return AVERROR_EXIT;
        if (wait_end && wait_end - av_gettime_relative() < 0)
            return AVERROR(EAGAIN);
        n = poll(p, rt->max_p, POLL_TIMEOUT_MS);
        if (n > 0) {
            timeout_cnt = 0;
            if (rt->rtp_handle) {
                ret = ffurl_read(rt->rtp_handle, buf, buf_size);
                if (ret > 0) {
                    return ret;
                }
            }
        } else if (n == 0 && ++timeout_cnt >= MAX_TIMEOUTS) {
            return AVERROR(ETIMEDOUT);
        } else if (n < 0 && errno != EINTR)
            return AVERROR(errno);
    }

}

int ff_gb28181_read_packet(AVFormatContext *s, AVPacket *pkt) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;

    int ret,len;
    int64_t wait_end = 0;
    int64_t first_queue_time = 0;
    int64_t queue_time = 0;
    RTPDemuxContext *rtpctx = NULL;
    unsigned int parse_timeout = 0; //1000*30
    unsigned int read_pkt_timeout = 0; //1000*30

    parse_timeout  = ff_gb28181_gettickcurmill();
    read_pkt_timeout = ff_gb28181_gettickcurmill();

retry:
    rtpctx = rt->transport_priv;
    wait_end = 0;
    first_queue_time = 0;
    queue_time = 0;
    if (!rtpctx)
        return -1;
    queue_time = ff_rtp_queued_packet_time(rtpctx);
    if (queue_time && (queue_time - first_queue_time < 0 ||
                       !first_queue_time)) {
        first_queue_time = queue_time;
    }

    if (first_queue_time) {
        wait_end = first_queue_time + s->max_delay;
    } else {
        wait_end = 0;
    }

    if (!rt->recvbuf) {
        rt->recvbuf = av_malloc(RECVBUF_SIZE);
        if (!rt->recvbuf)
            return AVERROR(ENOMEM);
    }

    /*
     * 1,if tcp
     *   1.1 read 1 byte
     *   1.2 read 3 byte

     * 2,if udp
     */
#if 0
    static FILE *fppkt_recv =  NULL;
    if(fppkt_recv == NULL) {
        fppkt_recv = fopen("recv_pkt.nal","wb+");
    }
    static FILE *fpudp_recv =  NULL;
    if(fpudp_recv == NULL) {
        fpudp_recv = fopen("recv_udp.rtp","wb+");
    }
#endif
    if(rt->transport_rtp_mask == GB28181_TRANSPORT_UDP) {
        len = ff_gb28181_rtp_read_packet(s,  rt->recvbuf, RECVBUF_SIZE,wait_end);

    }else {
        ret = ffurl_read_complete(rt->rtp_handle, rt->recvbuf, 2);
        if (ret != 2) {
            return AVERROR_EOF;
        }

        len = AV_RB16(rt->recvbuf);

        if (len > RECVBUF_SIZE || len < 8)
            goto retry;
        /* get the data */
        ret = ffurl_read_complete(rt->rtp_handle, rt->recvbuf, len);
        if (ret != len)
            return -1;
    }

    if (len == AVERROR(EAGAIN)) {
        av_log(s, AV_LOG_WARNING,
               "max delay reached. need to consume packet\n");
        ret = ff_rtp_parse_packet(rt->transport_priv, pkt, NULL, 0);
        if((ret < 0) && ((read_pkt_timeout + 200) > ff_gb28181_gettickcurmill())) {
            goto retry;
        }
        return ret;
    }
    if(len == AVERROR(ETIMEDOUT)) {
        av_log(s, AV_LOG_ERROR, "rtp_read_packet ETIMEDOUT failed.err=%s\n",av_err2str(len));
        return len;
    }
    read_pkt_timeout = ff_gb28181_gettickcurmill();
    if(len < 0) {
        av_log(s, AV_LOG_ERROR, "rtp_read_packet failed.ret=%d\n",len);
        return len;
    }
#if 0
    fwrite(rt->recvbuf, len,1,fpudp_recv);
#endif
    if (len > 0 && rt->transport_priv) {
        ff_rtp_check_and_send_back_rr(rt->transport_priv, rt->rtp_handle, NULL, len);
    }
    ret = ff_rtp_parse_packet(rt->transport_priv, pkt, &rt->recvbuf, len);
    if (ret < 0) {
        if((parse_timeout + TIMEOUT_PARSEPKT) > ff_gb28181_gettickcurmill()) {
            goto retry;
        }
    }
#if 0
    fwrite(pkt->data,pkt->size,1,fppkt_recv);
#endif
    return ret;
}

int ff_gb28181_media_close(AVFormatContext *s) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    if(rt->transport_priv) {
        ff_rtp_parse_close(rt->transport_priv);
        rt->transport_priv = NULL;
    }
    if (rt->rtp_handle) {
        ffurl_close(rt->rtp_handle);
        rt->rtp_handle = NULL;
    }
    if(rt->dynamic_handler->close  && rt->dynamic_handler && rt->dynamic_protocol_context){
        rt->dynamic_handler->close(rt->dynamic_protocol_context);
        av_free(rt->dynamic_protocol_context);
    }
    if(rt->recvbuf)
        av_free(rt->recvbuf);
    if(rt->p)
        av_free(rt->p);

    return 0;
}



