#pragma once

#include "os_config.h"
#include <string>

enum BM_MSG_ID
{
    BM_EXIT = 1,
    BM_DOWNLOAD_RSP,
    BM_REG_TIME_OUT,
    BM_INIT_CATALOG,
    BM_HTTP_INIT_CATALOG,
    BM_CATALOG_TIME_OUT,
    BM_CLEAR_CATALOG,
    BM_INIT_SUBSCRIBE,
    BM_SUBSCRIBE_TIME_OUT,
    BM_REFRESH_SUBSCRIBE,
    BM_INIT_RECORD,
    BM_RECORD_TIME_OUT,
    BM_GEN_HTTP_RSP,
    BM_INIT_INVITE,
    BM_INIT_BYE,
    BM_INVITE_CALL_RSP,
    BM_INVITE_TIME_OUT,
    BM_INIT_RTMP_PUBLISH,
    BM_INIT_RTMP_TRANS,
    BM_PLAY_BACK_NOTIFY,
    BM_STREAM_STAT_TIMER,
    BM_MEDIA_NODE_TIMER,
    BM_RTSP_SESSION_TIMER,
    BM_TRANS_KEY_TIMER,
    BM_NVR_INFO,
    BM_DOWN_NVR_FILE,
    BM_DOWN_NVR_RSP,
    BM_GB_SYNC_TIMER,
    BM_LIC_CHECK_TIMER,
    BM_ADD_NVR_DEV,
    BM_REGIST_NODE,
    BM_DETECT_TIMER,
    BM_SDK_PLAY,
    BM_SDK_PLAY_RSP,
    BM_SDK_STOP_PLAY,
    BM_SDK_DEL_CTX,
    BM_INIT_RETISTER,
    BM_INIT_MESSAGE,
    BM_INIT_RRETISTER,
    BM_INIT_PLAY,
    BM_INIT_PAUSE,
};

enum BM_SERVICE_TYPE
{
    BM_HTTP_SERVER = 1,
    BM_RTSP_ACCEPTOR,
    BM_RTSP_SERVER,
    BM_HTTP_DOWNLOAD,
    BM_HTTP_CLIENT,
    BM_GB_SERVER,
    BM_GB_CLIENT,
    BM_RTMP_PUBLISHER,
    BM_RTSP_CLIENT,
    BM_HIK_DEVICE,
    BM_DH_DEVICE,
    BM_GB_SOURCE,
    BM_HIK_SOURCE,
    BM_FILE_SOURCE,
};

enum TRASNSPORT
{
    EN_UDP = 0,
    EN_TCP_ACTIVE,
    EN_TCP_PASSIVE
};

enum WORK_MODE
{
    M_MASTER,
    M_SLAVE
};

class BmMsg
{
public:
    BmMsg();

    int m_sessinID;
    int m_msgID;
    int m_srcType;
    int m_srcID;
    int m_dstType;
    int m_dstID;

    int     m_intVal;
    string  m_strVal;
    void* m_ptr;
};
class ffBmMsg
{
public:
    ffBmMsg():m_msgID(0),isUdp(1),m_proto("gb28181"){};
    int m_msgID;
    int isUdp;
    string m_proto;
};