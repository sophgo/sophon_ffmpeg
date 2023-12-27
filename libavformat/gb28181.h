#ifndef AVFORMAT_GB28181_H
#define AVFORMAT_GB28181_H

#include <stdlib.h>
//#include <pthread.h>
#include "libavutil/thread.h"
#include <unistd.h>
#include "libavutil/avstring.h"
#include "libavutil/random_seed.h"
#include "libavutil/time.h"
#include "url.h"
#include "rtpproto.h"

#if HAVE_WINSOCK2_H
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include "netinet/in.h"
#endif

#if HAVE_POLL_H
#include <poll.h>
#endif
#include "internal.h"
#include "network.h"
#include "os_support.h"
#include <stdio.h>

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"

#include "url.h"
#include "rtpdec.h"

#include <stdbool.h>

#define TIMEOUT_PARSEPKT 1000*30 //30 second
#define POLL_TIMEOUT_MS 100
#define READ_PACKET_TIMEOUT_S 10
#define MAX_TIMEOUTS READ_PACKET_TIMEOUT_S * 1000 / POLL_TIMEOUT_MS
#define GB28181_DEFAULT_REORDERING_DELAY 100*1000

#define MAX_SIP_PORT_NUMS 200
#define SIP_SERVER_DEFAULT_PORT   5060
#define GB28181_SIP_PORT_MIN 5000
#define GB28181_SIP_PORT_MAX 6000
#define GB28181_RTP_PORT_MIN 20000
#define GB28181_RTP_PORT_MAX 30000
#define GB28181_SDP_PAYLOAD_TYPE 96
#define SIP_MSG_TIMEOUT 1000*5   //times,means ms
#define RECVBUF_SIZE 10 * RTP_MAX_PACKET_LENGTH


enum GB28181Transport {
    GB28181_TRANSPORT_UDP = 0,           /**< UDP/unicast */
    GB28181_TRANSPORT_TCP = 1,           /**< TCP; interleaved in RTSP */
};


typedef struct  RegistrationInfo
{
    const AVClass *av_class;
    //common info
    struct eXosip_t *pctx;
    unsigned int     lasttick;
    int              iheartbeat;      //heart interval (second)
    int              heartsn;
    int              expires;
    bool             bthreadexit;     //thread control
    bool             invite_answer;   //init:false   recv invite: true
    bool             register_success;//init:false   recv reg success: true


    char             proto[32];       //gb28181 or gb28181_playback
    //sip proxy info
    char             serverid[24];    //server 20 code
    char             serverip[20];    //server ip
    int              serverport;      //server port
    char             serverpasswd[48];//server pwd

    char             deviceid[24];
    char             devicetype[4];

    //sip client info
    char             localid[24];     //client 20 code
    char             localip[20];     //client ip
    char             localnatip[20];  //client portmapping ip
    char             localnatport[20];  //client portmapping port
    int              localportsip;    //client port for sip msg
    int              localportrtp;    //client port for rtp msg
    int              localportrtcp;   //client port for rtcp msg
    int              sockrtp;
    int              sockrtcp;
    int              sip_port_min;
    int              sip_port_max;

    int              cid;             //using in call up
    int              did;
    char             callid[256];

    //media info
    URLContext      *rtp_handle;      /**< RTP stream handle (if UDP) */
    int rtp_port_min, rtp_port_max;   //Minimum and maximum local UDP ports.
    int              buffer_size;
    int              transport_sip_mask;  //sip: tcp or udp
    int              transport_rtp_mask;  //rtp: tcp or udp
    void            *transport_priv;  //RTPDemuxContext
    char             media_ip[20];
    int              media_port;

    //play back
    bool             bFileEnd;    // true:file is end;  false:file is not end;
    char             playback_cmd[16];  //PLAY，PAUSE，TEARDOWN
    int              cseq;
    double           scale;        //1.0 正常速度播放
    char             begtime[32];
    char             endtime[32];


    const RTPDynamicProtocolHandler *dynamic_handler;
    PayloadContext *dynamic_protocol_context;

    struct pollfd   *p;               //Polling array for udp
    int              max_p;
    uint8_t*         recvbuf;

    struct AVPacketList *packet_buffer;
    struct AVPacketList *packet_buffer_end;

    int stream_index;
    char *sip_handle;

}RegistrationInfo;

//common
unsigned int ff_gb28181_gettickcurmill(void);
time_t convert_str_to_tm(char * str_time);
int ff_gb28181_parm_parse(AVFormatContext *rt);
int get_local_ip(char * ip);
int getLocalIp(char *ip);

//sip function
int ff_gb28181_sip_init(AVFormatContext *s);
int ff_gb28181_sip_uninit(AVFormatContext *s);
int ff_gb28181_sip_register(AVFormatContext *s);
int ff_gb28181_sip_unregister(AVFormatContext *s);
int ff_gb28181_sip_invite(AVFormatContext *s);
int ff_gb28181_sip_invite_ack(AVFormatContext *s);
int ff_gb28181_sip_invite_bye(RegistrationInfo *rt);
int ff_gb28181_sip_info_playback(AVFormatContext *s, const char *cmd);

//media function
int ff_gb28181_media_setup_rtp_channel(AVFormatContext *s);
int ff_gb28181_media_create_avstream(AVFormatContext *s);
int ff_gb28181_media_open_transport_ctx(AVFormatContext *s);
int ff_gb28181_rtp_read_packet(AVFormatContext *rt,uint8_t *buf, int buf_size, int64_t wait_end);
int ff_gb28181_read_packet(AVFormatContext *s, AVPacket *pkt);
int ff_gb28181_media_close(AVFormatContext *s);

#endif /* AVFORMAT_GB28181_H */
