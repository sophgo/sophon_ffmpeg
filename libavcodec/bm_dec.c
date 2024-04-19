
#include "libavutil/pixfmt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/time.h"
#if defined(BM1684)
#include "libavutil/hwcontext_bmcodec.h"
#endif

#include "avcodec.h"
#include "decode.h"
#include "libavutil/thread.h"
#if defined(BM1684)
#include "libavcodec/hwaccels.h"
#include "libavcodec/hwconfig.h"
#include "libavcodec/bm_codec.h"
#endif
#include "codec_internal.h"

#include <ctype.h>    /* toupper() */
#include <sys/time.h> /* timeval */

#include "bm_vpudec_interface.h"
#include "bm_dec.h"
#include "bmlib_runtime.h"
#include "config_components.h"


#define MAX_FRAME_IN_BUFFER      0
#define MIN_DATA_SIZE_IN_BUFFER  0x10000
#define USLEEP_CLOCK             1000 // here 1ms helpful for bm1684 decoding performance at small thread number

//externs
#ifdef BM_PCIE_MODE
extern int vdi_read_memory(unsigned long core_idx,unsigned long long addr,unsigned char *data,int len,int endian);
#endif
typedef struct BMCodecBuffer {
    BMDecContext   *ctx;
    BMVidFrame     *bmframe;
#if defined(BM1684)
    AVBmCodecFrame *hwpic;
#endif
#ifdef BM_PCIE_MODE
    uint8_t        *buf0;
    uint8_t        *buf1;
    uint8_t        *buf2;
    int            pcie_no_copyback;
#endif
    int            released;
    int            hw_accel_flag;
    BMHandleBuffer  *bm_handle_buffer;
} BMCodecBuffer;

typedef enum {
    STD_AVC,
    STD_VC1,
    STD_MPEG2,
    STD_MPEG4,
    STD_H263,
    STD_DIV3,
    STD_RV,
    STD_AVS,
    STD_THO = 9,
    STD_VP3,
    STD_VP8,
    STD_HEVC,
    STD_VP9,
    STD_AVS2,
    STD_SVAC,
    STD_MAX
} CodStd;

typedef struct {
    CodStd      codStd;
    unsigned int      mp4Class;
    unsigned int      codecId;
    unsigned int      fourcc;
} CodStdTab;

#ifndef MKTAG
#define MKTAG(a,b,c,d) (a | (b << 8) | (c << 16) | (d << 24))
#endif

static const CodStdTab codstd_tab[] = {
    { STD_AVC,          0, AV_CODEC_ID_H264,            MKTAG('H', '2', '6', '4') },
    { STD_AVC,          0, AV_CODEC_ID_H264,            MKTAG('X', '2', '6', '4') },
    { STD_AVC,          0, AV_CODEC_ID_H264,            MKTAG('A', 'V', 'C', '1') },
    { STD_AVC,          0, AV_CODEC_ID_H264,            MKTAG('V', 'S', 'S', 'H') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('H', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('X', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('T', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('L', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('V', 'X', '1', 'K') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('Z', 'y', 'G', 'o') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('H', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('I', '2', '6', '3') },    /* intel h263 */
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('H', '2', '6', '1') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('U', '2', '6', '3') },
    { STD_H263,         0, AV_CODEC_ID_H263,            MKTAG('V', 'I', 'V', '1') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('F', 'M', 'P', '4') },
    { STD_MPEG4,        5, AV_CODEC_ID_MPEG4,           MKTAG('D', 'I', 'V', 'X') },    // DivX 4
    { STD_MPEG4,        1, AV_CODEC_ID_MPEG4,           MKTAG('D', 'X', '5', '0') },
    { STD_MPEG4,        2, AV_CODEC_ID_MPEG4,           MKTAG('X', 'V', 'I', 'D') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', 'P', '4', 'S') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', '4', 'S', '2') },    //MPEG-4 version 2 simple profile
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG( 4 ,  0 ,  0 ,  0 ) },    /* some broken avi use this */
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('D', 'I', 'V', '1') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('B', 'L', 'Z', '0') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', 'P', '4', 'V') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('U', 'M', 'P', '4') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('W', 'V', '1', 'F') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('S', 'E', 'D', 'G') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('R', 'M', 'P', '4') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('3', 'I', 'V', '2') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('F', 'F', 'D', 'S') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('F', 'V', 'F', 'W') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('D', 'C', 'O', 'D') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', 'V', 'X', 'M') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('P', 'M', '4', 'V') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('S', 'M', 'P', '4') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('D', 'X', 'G', 'M') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('V', 'I', 'D', 'M') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', '4', 'T', '3') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('G', 'E', 'O', 'X') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('H', 'D', 'X', '4') }, /* flipped video */
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('D', 'M', 'K', '2') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('D', 'I', 'G', 'I') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('I', 'N', 'M', 'C') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('E', 'P', 'H', 'V') }, /* Ephv MPEG-4 */
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('E', 'M', '4', 'A') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('M', '4', 'C', 'C') }, /* Divio MPEG-4 */
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('S', 'N', '4', '0') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('V', 'S', 'P', 'X') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('U', 'L', 'D', 'X') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('G', 'E', 'O', 'V') },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG('S', 'I', 'P', 'P') }, /* Samsung SHR-6040 */
    { STD_DIV3,         0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('D', 'I', 'V', '3') }, /* default signature when using MSMPEG4 */
    { STD_DIV3,         0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('M', 'P', '4', '3') },
    { STD_DIV3,         0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('M', 'P', 'G', '3') },
    { STD_MPEG4,        1, AV_CODEC_ID_MSMPEG4V3,       MKTAG('D', 'I', 'V', '5') },
    { STD_MPEG4,        1, AV_CODEC_ID_MSMPEG4V3,       MKTAG('D', 'I', 'V', '6') },
    { STD_MPEG4,        5, AV_CODEC_ID_MSMPEG4V3,       MKTAG('D', 'I', 'V', '4') },
    { STD_DIV3,         0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('D', 'V', 'X', '3') },
    { STD_DIV3,         0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('A', 'P', '4', '1') },    //Another hacked version of Microsoft's MP43 codec.
    { STD_MPEG4,        0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('C', 'O', 'L', '1') },
    { STD_MPEG4,        0, AV_CODEC_ID_MSMPEG4V3,       MKTAG('C', 'O', 'L', '0') },    // not support ms mpeg4 v1, 2
    { STD_MPEG4,      256, AV_CODEC_ID_FLV1,            MKTAG('F', 'L', 'V', '1') }, /* Sorenson spark */
    { STD_VC1,          0, AV_CODEC_ID_WMV1,            MKTAG('W', 'M', 'V', '1') },
    { STD_VC1,          0, AV_CODEC_ID_WMV2,            MKTAG('W', 'M', 'V', '2') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG('M', 'P', 'G', '1') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG('M', 'P', 'G', '2') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('M', 'P', 'G', '2') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('M', 'P', 'E', 'G') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG('M', 'P', '2', 'V') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG('P', 'I', 'M', '1') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('P', 'I', 'M', '2') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG('V', 'C', 'R', '2') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG1VIDEO,      MKTAG( 1 ,  0 ,  0 ,  16) },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG( 2 ,  0 ,  0 ,  16) },
    { STD_MPEG4,        0, AV_CODEC_ID_MPEG4,           MKTAG( 4 ,  0 ,  0 ,  16) },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('D', 'V', 'R', ' ') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('M', 'M', 'E', 'S') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('L', 'M', 'P', '2') }, /* Lead MPEG2 in avi */
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('S', 'L', 'I', 'F') },
    { STD_MPEG2,        0, AV_CODEC_ID_MPEG2VIDEO,      MKTAG('E', 'M', '2', 'V') },
    { STD_VC1,          0, AV_CODEC_ID_WMV3,            MKTAG('W', 'M', 'V', '3') },
    { STD_VC1,          0, AV_CODEC_ID_VC1,             MKTAG('W', 'V', 'C', '1') },
    { STD_VC1,          0, AV_CODEC_ID_VC1,             MKTAG('W', 'M', 'V', 'A') },

    //  { STD_RV,           0, AV_CODEC_ID_RV30,            MKTAG('R','V','3','0') },
    //  { STD_RV,           0, AV_CODEC_ID_RV40,            MKTAG('R','V','4','0') },

    { STD_AVS,          0, AV_CODEC_ID_CAVS,            MKTAG('C','A','V','S') },
    { STD_AVS,          0, AV_CODEC_ID_AVS,             MKTAG('A','V','S','2') },
    { STD_VP3,          0, AV_CODEC_ID_VP3,             MKTAG('V', 'P', '3', '0') },
    { STD_VP3,          0, AV_CODEC_ID_VP3,             MKTAG('V', 'P', '3', '1') },
    //  { STD_THO,          0, AV_CODEC_ID_THEORA,          MKTAG('T', 'H', 'E', 'O') },
    { STD_VP8,          0, AV_CODEC_ID_VP8,             MKTAG('V', 'P', '8', '0') },
    //  { STD_VP9,          0, AV_CODEC_ID_VP9,             MKTAG('V', 'P', '9', '0') },
    //  { STD_VP6,              0, AV_CODEC_ID_VP6,             MKTAG('V', 'P', '6', '0') },
    //  { STD_VP6,              0, AV_CODEC_ID_VP6,             MKTAG('V', 'P', '6', '1') },
    //  { STD_VP6,              0, AV_CODEC_ID_VP6,             MKTAG('V', 'P', '6', '2') },
    //  { STD_VP6,              0, AV_CODEC_ID_VP6F,            MKTAG('V', 'P', '6', 'F') },
    //  { STD_VP6,              0, AV_CODEC_ID_VP6F,            MKTAG('F', 'L', 'V', '4') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('H', 'E', 'V', 'C') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('H', 'E', 'V', '1') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('H', 'V', 'C', '1') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('h', 'e', 'v', 'c') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('h', 'e', 'v', '1') },
    { STD_HEVC,         0, AV_CODEC_ID_HEVC,            MKTAG('h', 'v', 'c', '1') }
};

static enum AVPixelFormat pix_fmts[3] = {
    AV_PIX_FMT_BMCODEC,
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NONE
};
#ifdef BM_PCIE_MODE
typedef struct _TMP_AVframe
{
    char *data;
    int size;
    int inst_num;
    AVMutex av_mutex;
} TMP_AVframe;
TMP_AVframe tmp_av_frame = {NULL, 0, 0, AV_MUTEX_INITIALIZER};
#endif

static int bm_dec_framebuffer_withlock_unref(BMHandleBuffer **bm_handle_buffer){
    int ret = 0;
    if(!(*bm_handle_buffer))
        return -1;
    ff_mutex_lock(&(*bm_handle_buffer)->av_mutex);
    (*bm_handle_buffer)->ref_count--;

    if((*bm_handle_buffer)->ref_count == 0 && (*bm_handle_buffer)->handle){
        ret = bmvpu_dec_delete((*bm_handle_buffer)->handle);
        ff_mutex_unlock(&(*bm_handle_buffer)->av_mutex);
        ff_mutex_destroy(&(*bm_handle_buffer)->av_mutex);
        av_freep(bm_handle_buffer);
    }else{
        ff_mutex_unlock(&(*bm_handle_buffer)->av_mutex);
    }
    return ret;
}

static BMHandleBuffer *bm_dec_framebuffer_withlock_ref(BMHandleBuffer *bm_handle_buffer){
    if(!bm_handle_buffer)
        return NULL;
    ff_mutex_lock(&bm_handle_buffer->av_mutex);
    bm_handle_buffer->ref_count++;
    ff_mutex_unlock(&bm_handle_buffer->av_mutex);
    return bm_handle_buffer;
}

static av_cold int fourcc_to_mp4class(unsigned int fourcc)
{
    int   i;
    int   mp4Class = -1;
    unsigned char str[5];

    str[0] = toupper((int)fourcc);
    str[1] = toupper((int)(fourcc>>8));
    str[2] = toupper((int)(fourcc>>16));
    str[3] = toupper((int)(fourcc>>24));
    str[4] = '\0';

    for(i=0; i<sizeof(codstd_tab)/sizeof(codstd_tab[0]); i++) {
        if (codstd_tab[i].fourcc == (unsigned int)MKTAG(str[0], str[1], str[2], str[3]) ) {
            mp4Class = codstd_tab[i].mp4Class;
            break;
        }
    }

    return mp4Class;
}

static av_cold int fourcc_to_bmid(unsigned int fourcc)
{
    int   codStd = -1;
    int   i;

    char str[5];

    str[0] = toupper((int)fourcc);
    str[1] = toupper((int)(fourcc>>8));
    str[2] = toupper((int)(fourcc>>16));
    str[3] = toupper((int)(fourcc>>24));
    str[4] = '\0';

    for(i=0; i<sizeof(codstd_tab)/sizeof(codstd_tab[0]); i++) {
        if (codstd_tab[i].fourcc == (unsigned int)MKTAG(str[0], str[1], str[2], str[3])) {
            codStd = codstd_tab[i].codStd;
            break;
        }
    }

    return codStd;
}

static av_cold int codec_id_to_bm(int id)
{
    int bm_id = -1;
    int i;

    for(i=0; i<sizeof(codstd_tab)/sizeof(codstd_tab[0]); i++) {
        if (codstd_tab[i].codecId == id) {
            bm_id = codstd_tab[i].codStd;
            break;
        }
    }

    return bm_id;
}

static av_cold int codec_id_to_mp4class(int id)
{
    int   mp4class = -1;
    int   i;

    for(i=0; i<sizeof(codstd_tab)/sizeof(codstd_tab[0]); i++) {
        if (codstd_tab[i].codecId == id) {
            mp4class = codstd_tab[i].mp4Class;
            break;
        }
    }

    return mp4class;
}

static int codec_id_to_fourcc(int id)
{
    int   fourcc = 0;
    int   i;

    for(i=0; i<sizeof(codstd_tab)/sizeof(codstd_tab[0]); i++) {
        if (codstd_tab[i].codecId == id) {
            fourcc = codstd_tab[i].fourcc;
            break;
        }
    }
    return fourcc;
}
static enum AVPixelFormat bm_format_to_av_pixel_format(AVCodecContext *avctx, BMVidFrame* bmframe)
{
    enum AVPixelFormat fmt;

    switch (bmframe->frameFormat) {
    case 0:
        if(bmframe->cbcrInterleave == 1) {
            if(bmframe->nv21 == 0)
                fmt = AV_PIX_FMT_NV12;
            else if(bmframe->nv21 == 1)
                fmt = AV_PIX_FMT_NV21;
            else
                fmt = AV_PIX_FMT_NONE;
        }
        else if(bmframe->cbcrInterleave == 0)
            fmt = AV_PIX_FMT_YUV420P;
        else
            fmt = AV_PIX_FMT_NONE;
        break;
    case 1:
        fmt = AV_PIX_FMT_NV12;
        break;
    case 2:
        fmt = AV_PIX_FMT_YUYV422;
        break;
    case 3:
        fmt = AV_PIX_FMT_YUV422P;
        break;
    case 5:
        if(bmframe->cbcrInterleave)
            fmt = AV_PIX_FMT_P016BE;
        else
            fmt = AV_PIX_FMT_YUV420P16BE;
        break;
    case 6:
        if(bmframe->cbcrInterleave)
            fmt = AV_PIX_FMT_P016LE;
        else
            fmt = AV_PIX_FMT_YUV420P16LE;
        break;
    default:
        fmt = AV_PIX_FMT_NONE;
        break;
    }

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] pixel_fmt:%s\n",
           __func__, __LINE__, av_get_pix_fmt_name(fmt));

    return fmt;
}

static int is_fill_pkg_info(int bm_id)
{
    int ret = 0;
    if(bm_id == STD_DIV3)
        ret = 1;
    return ret;
}

static av_cold int bm_decode_init(AVCodecContext *avctx)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    BMVidDecParam param;
    BMVidCodHandle handle;
#if defined(BM1684)
    AVBmCodecDeviceContext *bmcodec_device_hwctx = NULL;
#endif
    int bm_id = -1;
    int ret;

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    bmctx->avctx = avctx;

    bmctx->old_pix_fmt = AV_PIX_FMT_NONE;
    if (bmctx->cbcr_interleave == 1) {
        avctx->sw_pix_fmt = pix_fmts[1] = AV_PIX_FMT_NV12;
    }
    else if(bmctx->cbcr_interleave == 2) {
        avctx->sw_pix_fmt = pix_fmts[1] = AV_PIX_FMT_NV21;
    }
    else {
        avctx->sw_pix_fmt = pix_fmts[1] = AV_PIX_FMT_YUV420P;
    }

    ret = ff_get_format(avctx, pix_fmts);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed.\n");
        return ret;
    }
    av_log(avctx, AV_LOG_DEBUG, "ff_get_format: %s.\n", av_get_pix_fmt_name(ret));

    avctx->pix_fmt = ret;

    bmctx->hw_accel = 0;
#if defined(BM1684)
    ret = bmcodec_get_device_hwctx(avctx, &bmcodec_device_hwctx);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "bmcodec_get_device_hwctx failed\n");
        return ret;
    }
    if (bmcodec_device_hwctx) {
        bmctx->hw_accel = 1;
        bmctx->soc_idx = bmcodec_device_hwctx->device_idx;
        av_log(avctx, AV_LOG_DEBUG, "bmcodec_device_hwctx: device_idx=%d\n", bmcodec_device_hwctx->device_idx);
    }
#endif

#ifndef BM_PCIE_MODE
    bmctx->soc_idx = 0;
#endif

    av_log(avctx, AV_LOG_DEBUG, "bmctx->hw_accel=%d\n", bmctx->hw_accel);
    if (bmctx->hw_accel == 0) {
        avctx->sw_pix_fmt = AV_PIX_FMT_NV12;
        avctx->pix_fmt    = AV_PIX_FMT_NV12;
    }

    memset(&param, 0, sizeof(param));

    bm_id = fourcc_to_bmid(avctx->codec_tag);
    if(bm_id == -1)
        bm_id = codec_id_to_bm(avctx->codec->id);

    param.streamFormat = bm_id;
    bmctx->bm_id = bm_id;

    av_log(avctx, AV_LOG_INFO, "bm decoder id: %d\n", bm_id);

#ifdef BM_PCIE_MODE
    param.pcie_board_id = bmctx->soc_idx;
    av_log(avctx, AV_LOG_INFO, "sophon device: %d\n", bmctx->soc_idx);
    if (bmctx->hw_accel==0) {
        param.pcie_no_copyback = bmctx->pcie_no_copyback;
        av_log(avctx, AV_LOG_INFO, "don't copy back image? : %d\n", bmctx->pcie_no_copyback);
    }
#endif

    bmctx->mp4class = 0;
    if(bm_id == STD_MPEG4) {
        int mp4class = fourcc_to_mp4class(avctx->codec_tag);
        if(mp4class == -1)
            mp4class = codec_id_to_mp4class(bm_id);
        if(mp4class != -1)
            param.mp4class = mp4class;
        bmctx->mp4class = param.mp4class;
        av_log(avctx, AV_LOG_INFO, "bm mp4class: %d\n", param.mp4class);
    }

#if defined(BM1684)
    if (bmctx->output_format != BMDEC_OUTPUT_UNMAP && bmctx->output_format != BMDEC_OUTPUT_COMPRESSED) {
        av_log(avctx, AV_LOG_ERROR, "Invalid output_format: %d. Please choose one of %d, %d\n",
               bmctx->output_format, BMDEC_OUTPUT_UNMAP, BMDEC_OUTPUT_COMPRESSED);
        return AVERROR(EINVAL);
    }
#else
    if (bmctx->output_format != BMDEC_OUTPUT_UNMAP && bmctx->output_format != BMDEC_OUTPUT_TILED) {
        av_log(avctx, AV_LOG_ERROR, "Invalid output_format: %d. Please choose one of %d, %d\n",
               bmctx->output_format, BMDEC_OUTPUT_UNMAP, BMDEC_OUTPUT_TILED);
        return AVERROR(EINVAL);
    }
#endif

    av_log(avctx, AV_LOG_INFO, "bm output format: %d\n", bmctx->output_format);

    param.cbcrInterleave = bmctx->cbcr_interleave;
    param.wtlFormat      = bmctx->output_format;

    if (bmctx->secondary_axi == 0 || bm_id == STD_VC1){
        param.secondaryAXI = 0;
    }
    else if(bmctx->secondary_axi == 1){
        param.secondaryAXI = 0xf;
    }
    else if(bmctx->secondary_axi == 2){
        if((avctx->width > 1920 && avctx->width <= 4096) && (avctx->height <= 2400))
            param.secondaryAXI = 0x7;
        else if(avctx->width > 4096 || avctx->height > 2400)
            param.secondaryAXI = 0x3;
        else
            param.secondaryAXI = 0xf;
    }

    param.streamBufferSize = bmctx->size_input_buffer;
    param.bsMode = bmctx->mode_bitstream;
    param.frameDelay = bmctx->frame_delay;
#ifdef BM_PCIE_MODE
    param.enable_cache = 0;
    ff_mutex_lock(&tmp_av_frame.av_mutex);
    tmp_av_frame.inst_num += 1;
    if (tmp_av_frame.data == NULL) {
        tmp_av_frame.size = FFALIGN(avctx->width,64) * FFALIGN(avctx->height,32) * 3 / 2;
        if (tmp_av_frame.size <= 0)
            tmp_av_frame.size = 1920*1088*3/2;
        tmp_av_frame.data = av_mallocz(tmp_av_frame.size);
    }
    ff_mutex_unlock(&tmp_av_frame.av_mutex);
#else
    param.enable_cache = bmctx->enable_cache;
#endif
    av_log(avctx, AV_LOG_INFO, "mode bitstream: %d, frame delay: %d\n", bmctx->mode_bitstream, bmctx->frame_delay);
    param.extraFrameBufferNum = bmctx->extra_frame_buffer_num;
    param.skip_mode = bmctx->skip_non_idr;
    param.perf = bmctx->perf;
    param.core_idx = bmctx->core_idx;
    param.cmd_queue_depth = bmctx->dec_cmd_queue;

    ret = bmvpu_dec_create(&handle, param);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "bmvpu_dec_create failed\n");
        ret = AVERROR_INVALIDDATA;
    }
    bmctx->handle = handle;
    bmctx->endof_flag = 0;
    bmctx->pkg_num_inbuf = 0;
    bmctx->first_frame = 0;
    bmctx->pkt_flag = 0;
    bmctx->first_frame_get = 0;
    bmctx->pts_offset = 0;

    BMHandleBuffer *bm_handle_buffer = (BMHandleBuffer*)av_mallocz(sizeof(BMHandleBuffer));
    ff_mutex_init(&(bm_handle_buffer->av_mutex), NULL);
    bm_handle_buffer->handle    = handle;
    bm_handle_buffer->ref_count = 1;
    bmctx->bm_handle_buffer = bm_handle_buffer;
    if (is_fill_pkg_info(bm_id)) {
        bmctx->header_buf = malloc(8192);
        bmctx->header_size = 8192;
    } else {
        bmctx->header_buf = NULL;
        bmctx->header_size = 0;
    }

    av_log(avctx, AV_LOG_DEBUG, "perf: %d\n", bmctx->perf);
    if (bmctx->perf) {
        bmctx->total_time  = 0.0f;
        bmctx->total_frame = 0L;
    }

    av_log(avctx, AV_LOG_DEBUG,
           "init options: mode, %d, frame delay, %d, output format, %d, extra frame buffer number: %d, extra_data_flag: %d\n",
            bmctx->mode_bitstream, bmctx->frame_delay, bmctx->output_format, bmctx->extra_frame_buffer_num, bmctx->extra_data_flag);

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return ret;
}

static void bm_buffer_release(void *opaque, uint8_t *data)
{
    BMCodecBuffer *buffer = opaque;

    if (buffer == NULL)
        return;

    //This is buffer you can use it before BMDEC_CLOSE.
    //if the status is BMDEC_CLOSE. the buffer unsafe.
    if (buffer->bm_handle_buffer->handle && buffer->bmframe && !buffer->released) {
        bmvpu_dec_clear_output(buffer->bm_handle_buffer->handle, buffer->bmframe);
        free(buffer->bmframe);
        buffer->bmframe = NULL;
     }

#if defined(BM1684)
    if (buffer->hwpic && !buffer->released){
        if (buffer->hwpic->buffer)
            av_free(buffer->hwpic->buffer);
        av_free(buffer->hwpic);
    }
#endif
    if (!buffer->hw_accel_flag && !buffer->released) {
#ifdef BM_PCIE_MODE
        if(!buffer->released &&  buffer->pcie_no_copyback == 0 && buffer->buf0)
            av_freep(&buffer->buf0);
#endif
    }

    if(buffer->bm_handle_buffer && !buffer->released)
        bm_dec_framebuffer_withlock_unref(&buffer->bm_handle_buffer);
    buffer->released = 1;

    av_freep(&buffer);
}

static void bm_buffer_release2(void *opaque, uint8_t *data)
{
}

static int select_frame_type(AVCodecContext *avctx, BMVidFrame* bmframe)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    int pict_type = AV_PICTURE_TYPE_NONE;

    if (bmctx->bm_id == STD_VC1) { /* For VC1 decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_I:             /* I picture */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
        case BM_PIC_TYPE_VC1_P_SKIP   : /* VC1 P skip picture (VC1 only) */
            pict_type = AV_PICTURE_TYPE_P;
            break;
        case BM_PIC_TYPE_VC1_B        : /* VC1 B picture (VC1 only) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        case BM_PIC_TYPE_VC1_BI       : /* VC1 BI picture (VC1 only) */
            pict_type = AV_PICTURE_TYPE_BI;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else if (bmctx->bm_id == STD_AVS2) { /* For AVS2 decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_I:             /* I picture */
        case BM_PIC_TYPE_AVS2_G:        /* G(round) picture in AVS2, display TODO */
        case BM_PIC_TYPE_AVS2_GB:       /* G(round)B picture in AVS2, no-display TODO */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
        case BM_PIC_TYPE_AVS2_F       : /* F picture in AVS2 */
        case BM_PIC_TYPE_AVS2_S       : /* S picture in AVS2, refer to G/GB picture */
        case BM_PIC_TYPE_B            : /* B picture (except VC1) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else if (bmctx->bm_id == STD_VP9) { /* For VP9 decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_IDR:           /* H.264/H.265 IDR picture */
        case BM_PIC_TYPE_I:             /* I picture */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
        case BM_PIC_TYPE_REPEAT       : /* Repeat frame (VP9 only) */
            pict_type = AV_PICTURE_TYPE_P;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else if (bmctx->bm_id == STD_SVAC) { /* For SVAC decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_IDR:           /* H.264/H.265 IDR picture */
        case BM_PIC_TYPE_KEY:           /* KEY frame for SVAC */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_INTER        : /* Inter frame for SVAC */
            pict_type = AV_PICTURE_TYPE_P;
            break;
        case BM_PIC_TYPE_B            : /* B picture (except VC1) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else if (bmctx->bm_id == STD_MPEG2) { /* For MPEG2 decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_I:             /* I picture */
        case BM_PIC_TYPE_D            : /* D picture in MPEG2 that is only composed of DC coefficients (MPEG2 only) */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
            pict_type = AV_PICTURE_TYPE_P;
            break;
        case BM_PIC_TYPE_B            : /* B picture (except VC1) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else if (bmctx->bm_id == STD_MPEG4) { /* For MPEG4 decoder */
        switch (bmframe->picType) {
        case BM_PIC_TYPE_I:             /* I picture */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
        case BM_PIC_TYPE_MP4_P_SKIP_NOT_CODED : /* Not Coded P Picture in MPEG4 packed mode */
            pict_type = AV_PICTURE_TYPE_P;
            break;
        case BM_PIC_TYPE_B            : /* B picture (except VC1) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        case BM_PIC_TYPE_S            : /* S picture in MPEG4 that is an acronym of Sprite and used for GMC (MPEG4 only)*/
            pict_type = AV_PICTURE_TYPE_S;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    } else {
        switch (bmframe->picType) {
        case BM_PIC_TYPE_IDR:           /* H.264/H.265 IDR picture */
        case BM_PIC_TYPE_I:             /* I picture */
            pict_type = AV_PICTURE_TYPE_I;
            break;
        case BM_PIC_TYPE_P            : /* P picture */
        case BM_PIC_TYPE_B            : /* B picture (except VC1) */
            pict_type = AV_PICTURE_TYPE_B;
            break;
        default:
            pict_type = AV_PICTURE_TYPE_NONE;
            break;
        }
    }

    return pict_type;
}

static int bm_fill_frame(AVCodecContext *avctx, BMVidFrame* bmframe, AVFrame* frame)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    BMCodecBuffer* buffer = NULL;
    const AVPixFmtDescriptor *desc;

#if defined(BM1684)
    AVBmCodecFrame* hwpic = NULL;
#endif
#ifdef BM_PCIE_MODE
    BMVidCodHandle handle = bmctx->handle;
    int coreIdx = bmvpu_dec_get_core_idx(handle);
#endif
    int i, ret = 0;

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] enter\n", __func__, __LINE__);

    if (frame->buf[0] != NULL) {
        av_log(avctx, AV_LOG_ERROR, "can't fill frame buffer, please check it.\n");
        return -1;
    }

    if (avctx->width != bmframe->width ||
        avctx->height != bmframe->height ||
        bmctx->old_pix_fmt != bm_format_to_av_pixel_format(avctx, bmframe)) {
        avctx->width = bmframe->width;
        avctx->height = bmframe->height;
        avctx->coded_width = bmframe->coded_width;
        avctx->coded_height = bmframe->coded_height;
        bmctx->old_pix_fmt = bm_format_to_av_pixel_format(avctx, bmframe);

        if (bmctx->hw_accel) {
            av_log(avctx, AV_LOG_DEBUG, "old_pix_fmt: %s\n",
                   av_get_pix_fmt_name(bmctx->old_pix_fmt));
            avctx->sw_pix_fmt = bmctx->old_pix_fmt;
            pix_fmts[1] = bmctx->old_pix_fmt;

            ret = ff_get_format(avctx, pix_fmts);
            if (ret < 0)
                return -1;

            avctx->pix_fmt = ret;
        } else {
            avctx->pix_fmt = bmctx->old_pix_fmt;
        }
    }

    frame->pict_type = select_frame_type(avctx, bmframe);
    frame->width  = avctx->width;
    frame->height = avctx->height;
    frame->format = avctx->pix_fmt;
    frame->colorspace  = avctx->colorspace;
    frame->color_range = avctx->color_range;
    frame->color_primaries = avctx->color_primaries;
    frame->color_trc = avctx->color_trc;
    frame->chroma_location = avctx->chroma_sample_location;
    desc = av_pix_fmt_desc_get(avctx->pix_fmt);

    av_log(avctx, AV_LOG_TRACE, "pixel format: %s, color space: %s, color range: %s\n",
           av_get_pix_fmt_name(avctx->pix_fmt),
           av_color_space_name(avctx->colorspace),
           av_color_range_name(avctx->color_range));

#ifdef BM_PCIE_MODE
    if (bmframe->width==0 || bmframe->height== 0)
#else
    if (bmframe->width==0 || bmframe->height== 0 || bmframe->buf[0]==NULL)
#endif
        av_log(avctx, AV_LOG_WARNING, "please check the output!!!\n");

    buffer = av_mallocz(sizeof(BMCodecBuffer));
    //av_log(avctx, AV_LOG_INFO, "buffer addr: %p, format: %d, frameIdx: %d\n", buffer, avctx->pix_fmt, bmframe->frameIdx);
    if (!buffer) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    buffer->ctx      = bmctx;
    buffer->bmframe  = bmframe;
    buffer->hw_accel_flag = bmctx->hw_accel;

#if defined(BM1684)
    buffer->hwpic    = NULL;

    hwpic = av_mallocz(sizeof(AVBmCodecFrame));
    if (hwpic == NULL) {
        av_log(avctx, AV_LOG_ERROR, "av_mallocz failed\n");
        return AVERROR(ENOMEM);
    }

    hwpic->type = 0;
    hwpic->buffer = NULL;
    hwpic->coded_width = avctx->coded_width;
    hwpic->coded_height = avctx->coded_height;

    buffer->hwpic = hwpic;
    //buffer->bm_handle_buffer = bmctx->bm_handle_buffer;
    buffer->bm_handle_buffer = bm_dec_framebuffer_withlock_ref(bmctx->bm_handle_buffer);
    if(!buffer->bm_handle_buffer){
        ret = AVERROR(ENOMEM);
        goto fail;
    }

 #endif

    /* For bm hw accel framework */
    if (bmctx->hw_accel) {
#if defined(BM1684)

#ifndef BM_PCIE_MODE
        /* vpu use reserved memory, we have to build a fake ion_buffer */
        bm_device_mem_t *data = (bm_device_mem_t *)av_malloc(sizeof(bm_device_mem_t));
        if (data == NULL) {
            av_log(avctx, AV_LOG_ERROR, "av_malloc failed\n");
            return AVERROR(ENOMEM);
        }

        data->u.device.dmabuf_fd = -1;   // vpu reserved memory has not ion fd
        data->u.device.device_addr = (uint64_t)bmframe->buf[4];
        data->size = bmframe->size;

        hwpic->buffer = data;  //member buffer is only for soc, pcie do not use.
        hwpic->buffer_vaddr = bmframe->buf[0];
#endif

        if (bmctx->output_format == BMDEC_OUTPUT_COMPRESSED)
            hwpic->maptype = 1;
        else
            hwpic->maptype = 0;

        for (i=0; i<4; i++) {
            frame->data[i]     =
            hwpic->data[i]     = bmframe->buf[4+i];

            frame->linesize[i] =
            hwpic->linesize[i] = bmframe->stride[i];
        }

        frame->data[4] = (uint8_t*)hwpic;
        frame->buf[0] = av_buffer_create(bmframe->buf[4],
                                        bmframe->stride[0]*bmframe->height,
                                        bm_buffer_release,
                                        buffer,
                                        AV_BUFFER_FLAG_READONLY);

        frame->buf[1] = av_buffer_create(bmframe->buf[5],
                                        bmframe->stride[1]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                        bm_buffer_release2,
                                        NULL,
                                        AV_BUFFER_FLAG_READONLY);

        if (bmframe->cbcrInterleave == 0) {
            frame->buf[2] = av_buffer_create(bmframe->buf[6],
                                            bmframe->stride[2]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                            bm_buffer_release2,
                                            NULL,
                                            AV_BUFFER_FLAG_READONLY);
            if (!frame->buf[2]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
        }

        if (avctx->hw_frames_ctx) {
            frame->hw_frames_ctx = av_buffer_ref(avctx->hw_frames_ctx);
            if (!frame->hw_frames_ctx)
                return AVERROR(ENOMEM);
        }
#endif
    } else {
#ifndef BM_PCIE_MODE
        frame->buf[0] = av_buffer_create(bmframe->buf[0],
                                        bmframe->stride[0]*bmframe->height,
                                        bm_buffer_release,
                                        buffer,
                                        AV_BUFFER_FLAG_READONLY);
        frame->data[0] = bmframe->buf[0];

        frame->buf[1] = av_buffer_create(bmframe->buf[1],
                                        bmframe->stride[1]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                        bm_buffer_release2,
                                        NULL,
                                        AV_BUFFER_FLAG_READONLY);
        frame->data[1] = bmframe->buf[1];

        if (bmframe->cbcrInterleave == 0) {
            frame->buf[2] = av_buffer_create(bmframe->buf[2],
                                            bmframe->stride[2]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                            bm_buffer_release2,
                                            NULL,
                                            AV_BUFFER_FLAG_READONLY);
            frame->data[2] = bmframe->buf[2];
            if (!frame->buf[2]) {
                ret = AVERROR(ENOMEM);
                goto fail;

            }
        }
#else
        //copy decoded data from devices memory to host memory
        if(bmframe->size > 0 && bmframe->size <= 8192*4096*3) {
            //optimize the framebuffer cdma copy.
            if(bmctx->pcie_no_copyback)
                buffer->buf0 = tmp_av_frame.data;
            else
                buffer->buf0 = av_malloc(bmframe->size);
            buffer->buf1 = buffer->buf0 + (unsigned int)(bmframe->buf[5] - bmframe->buf[4]);
            if(bmframe->cbcrInterleave == 0)
                buffer->buf2 = buffer->buf0 + (unsigned int)(bmframe->buf[6] - bmframe->buf[4]);
        } else {
            buffer->buf0 = NULL;
            av_log(avctx, AV_LOG_ERROR, "the frame buffer size maybe error..\n");
            goto fail2;
        }

        //VDI_LITTLE_ENDIAN(64bit LE)=0
        if( !bmctx->pcie_no_copyback && bmctx->output_format == BMDEC_OUTPUT_UNMAP) {
            vdi_read_memory(coreIdx, (unsigned long long)bmframe->buf[4], buffer->buf0, bmframe->size, 0);
        }

        buffer->pcie_no_copyback = bmctx->pcie_no_copyback;
        frame->buf[0] = av_buffer_create(buffer->buf0,
                                        bmframe->stride[0]*bmframe->height,
                                        bm_buffer_release,
                                        buffer,
                                        AV_BUFFER_FLAG_READONLY);
        frame->data[0] = buffer->buf0;

        frame->buf[1] = av_buffer_create(buffer->buf1,
                                        bmframe->stride[1]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                        bm_buffer_release2,
                                        NULL,
                                        AV_BUFFER_FLAG_READONLY);
        frame->data[1] = buffer->buf1;

        if(bmframe->cbcrInterleave == 0) {
            frame->buf[2] = av_buffer_create(buffer->buf2,
                                            bmframe->stride[2]*AV_CEIL_RSHIFT(bmframe->height, desc->log2_chroma_h),
                                            bm_buffer_release2,
                                            NULL,
                                            AV_BUFFER_FLAG_READONLY);
            frame->data[2] = buffer->buf2;
            if (!frame->buf[2]) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
        }
#endif

#if defined(BM1684)
        if (hwpic)
            frame->buf[4] = av_buffer_create(hwpic,
                                             sizeof(AVBmCodecFrame),
                                             bm_buffer_release2,
                                             NULL,
                                             AV_BUFFER_FLAG_READONLY);  // hack: pass coded_width/coded_height to scale_bm filter
#endif
    }

    if (!frame->buf[0] || !frame->buf[1]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }


    if (bmctx->hw_accel) {
#if defined(BM1684)
        av_log(avctx, AV_LOG_TRACE, "video decoder\n");
        for (i=0; i<4; i++)
            av_log(avctx, AV_LOG_TRACE, "\tdata %d: %p\n", i, hwpic->data[i]);

        for (i=0; i<4; i++)
            av_log(avctx, AV_LOG_TRACE, "\tlinesize %d: %d\n", i, hwpic->linesize[i]);

        av_log(avctx, AV_LOG_TRACE, "\twidth : %d\n", frame->width);
        av_log(avctx, AV_LOG_TRACE, "\theight: %d\n", frame->height);
#endif
    } else {
        for(i=0; i<4; i++){
            frame->data[4+i]    = bmframe->buf[i+4];
            frame->linesize[i]   = bmframe->stride[i];
            frame->linesize[4+i] = bmframe->stride[i+4];
        }
    }

    /* for to get tiled & compressed format 100: tiled, 101: compressed. */
    if (!bmctx->hw_accel) {
        frame->channel_layout = bmctx->output_format;
        frame->channels = av_get_channel_layout_nb_channels(frame->channel_layout);
    }

    frame->pts = bmframe->pts;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
    frame->pkt_pts = bmframe->pts;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    frame->pkt_dts = bmframe->dts;

    if (bmctx->perf)
        bmctx->total_frame++;

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return 0;

fail:
    av_buffer_unref(&frame->buf[0]);
    av_buffer_unref(&frame->buf[1]);
    av_buffer_unref(&frame->buf[2]);
#ifdef BM_PCIE_MODE
fail2:
#endif
    if(buffer->bm_handle_buffer){
        ret = bm_dec_framebuffer_withlock_unref(&buffer->bm_handle_buffer);
        if(ret < 0){
            av_log(avctx, AV_LOG_ERROR, "buffer->bm_handle_buffer NULL\n");
        }
    }
    av_freep(buffer);
    ret = bmvpu_dec_clear_output(bmctx->handle, bmframe);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to release output buffer\n");
        ret = AVERROR_EXTERNAL;
    }
    free(bmframe);
    bmframe = NULL;

    av_log(avctx, AV_LOG_TRACE, "[%s,%d] leave\n", __func__, __LINE__);

    return ret;
}

#define PUT_BYTE(_p, _b) \
    *_p++ = (unsigned char)_b;

#define PUT_BUFFER(_p, _buf, _len) \
    memcpy(_p, _buf, _len); \
    _p += _len;

#define PUT_LE32(_p, _var) \
    *_p++ = (unsigned char)((_var)>>0);  \
    *_p++ = (unsigned char)((_var)>>8);  \
    *_p++ = (unsigned char)((_var)>>16); \
    *_p++ = (unsigned char)((_var)>>24);
#define PUT_BE32(_p, _var) \
    *_p++ = (unsigned char)((_var)>>24);  \
    *_p++ = (unsigned char)((_var)>>16);  \
    *_p++ = (unsigned char)((_var)>>8); \
    *_p++ = (unsigned char)((_var)>>0);

#define PUT_LE16(_p, _var) \
    *_p++ = (unsigned char)((_var)>>0);  \
    *_p++ = (unsigned char)((_var)>>8);

#define PUT_BE16(_p, _var) \
        *_p++ = (unsigned char)((_var)>>8);  \
        *_p++ = (unsigned char)((_var)>>0);
#define RCV_V2
static int build_seq_header(unsigned char *buf, AVCodecContext *avctx, AVPacket *pkt, int bm_id)
{
    unsigned char* extradata = avctx->extradata;
    int extradata_size = avctx->extradata_size;
    unsigned char* p = extradata;
    unsigned char* a = p + extradata_size - 3;//4 - ((long) p & 3);
    int size = 0;
    int sps, i, pps, nal;
    int fourcc;
    int framerate = 0;

    fourcc = avctx->codec_tag;
    if (!fourcc)
        fourcc = codec_id_to_fourcc(avctx->codec_id);

    if (avctx->framerate.den && avctx->framerate.num)
        framerate = (int)((double)avctx->framerate.num/(double)avctx->framerate.den);

    switch (bm_id) {
    case STD_AVC:
    case STD_AVS:
        if (extradata_size > 1 && extradata && extradata[0] == 0x01) {
            // check mov/mo4 file format stream
            extradata += 5;

            sps = (*extradata++ & 0x1f); // Number of sps

            for (i = 0; i < sps; i++) {
                nal = (*extradata << 8) + *(extradata + 1) + 2;
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x01);
                if(extradata+nal > avctx->extradata + avctx->extradata_size)
                    break;
                PUT_BUFFER(buf, extradata+2, nal-2);
                extradata += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }

            pps = *(extradata++); // number of pps
            for (i = 0; i < pps; i++)
            {
                nal = (*extradata << 8) + *(extradata + 1) + 2;
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x00);
                PUT_BYTE(buf, 0x01);
                if(extradata+nal > avctx->extradata + avctx->extradata_size)
                    break;
                PUT_BUFFER(buf, extradata+2, nal-2);
                extradata += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }
        }
        else if(extradata_size > 3) {
            size = 0;// return to meaning of invalid stream data;
            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1)  {
                    // find startcode
                    size = avctx->extradata_size;
                    PUT_BUFFER(buf, avctx->extradata, size);
                    break;
                }
            }
        }
        break;
    case STD_HEVC:
        if (extradata_size > 1 && extradata && extradata[0] == 0x01) {
            static const unsigned char nalu_header[4] = { 0, 0, 0, 1 };
            int numOfArrays = 0;
            unsigned short numNalus = 0;
            unsigned short nalUnitLength = 0;
            unsigned int offset = 0;

            p += 22;

            numOfArrays = *p++;

            while(numOfArrays--) {
                p++;   // NAL type
                numNalus = (*p << 8) + *(p + 1);
                p+=2;
                for(i = 0;i < numNalus;i++)
                {
                    nalUnitLength = (*p << 8) + *(p + 1);
                    p+=2;
                    if(p+nalUnitLength <= extradata + extradata_size)
                    {
                        memcpy(buf + offset, nalu_header, 4);
                        offset += 4;
                        memcpy(buf + offset, p, nalUnitLength);
                        offset += nalUnitLength;
                        p += nalUnitLength;
                    }
                    else
                        return 0;
                }
            }

            size = offset;
        }
        else if(extradata_size > 3)
        {
            size = 0;// return to meaning of invalid stream data;

            for (; p < a; p++)
            {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) // find startcode
                {
                    size = avctx->extradata_size;
                    PUT_BUFFER(buf, extradata, size);
                    break;
                }
            }
        }
        break;
    case STD_VC1:
        if (!fourcc)
            return 0;
        if (fourcc == MKTAG('W', 'V', 'C', '1') || fourcc == MKTAG('W', 'M', 'V', 'A')) //VC AP
        {
            size = extradata_size;
            PUT_BUFFER(buf, extradata, size);
            //if there is no seq startcode in pbMetatData. VPU will be failed at seq_init stage.
        }
        else
        {
#ifdef RCV_V2
            PUT_LE32(buf, ((0xC5 << 24)|0));
            size += 4; //version
            PUT_LE32(buf, extradata_size);
            size += 4;
            PUT_BUFFER(buf, extradata, extradata_size);
            size += extradata_size;
            PUT_LE32(buf, avctx->height);
            size += 4;
            PUT_LE32(buf, avctx->width);
            size += 4;
            PUT_LE32(buf, 12);
            size += 4;
            PUT_LE32(buf, 2 << 29 | 1 << 28 | 0x80 << 24 | 1 << 0);
            size += 4; // STRUCT_B_FRIST (LEVEL:3|CBR:1:RESERVE:4:HRD_BUFFER|24)
            PUT_LE32(buf, avctx->bit_rate);
            size += 4; // hrd_rate
            PUT_LE32(buf, framerate);
            size += 4; // frameRate
#else  //RCV_V1
            PUT_LE32(buf, (0x85 << 24) | 0x00);
            size += 4; //frames count will be here
            PUT_LE32(buf, extradata_size);
            size += 4;
            PUT_BUFFER(buf, extradata, extradata_size);
            size += extradata_size;
            PUT_LE32(buf, avctx->height);
            size += 4;
            PUT_LE32(buf, avctx->width);
            size += 4;
#endif
        }

        break;

    case STD_DIV3:
        if (!extradata_size) {
            PUT_LE32(buf, MKTAG('C', 'N', 'M', 'V')); //signature 'CNMV'
            PUT_LE16(buf, 0x00);                      //version
            PUT_LE16(buf, 0x20);                      //length of header in bytes
            PUT_LE32(buf, MKTAG('D', 'I', 'V', '3')); //codec FourCC
            PUT_LE16(buf, avctx->width);                //width
            PUT_LE16(buf, avctx->height);               //height
            PUT_LE32(buf, avctx->framerate.num);      //frame rate
            PUT_LE32(buf, avctx->framerate.den);      //time scale(?)
            PUT_LE32(buf, avctx->frame_number);      //number of frames in file
            PUT_LE32(buf, 0); //unused
            size += 32;
            return size;
        }

        PUT_BE32(buf, extradata_size);
        size += 4;

        PUT_BUFFER(buf, extradata, extradata_size);
        size += extradata_size;
        break;
    case STD_VP8:
        PUT_LE32(buf, MKTAG('D', 'K', 'I', 'F')); //signature 'DKIF'
        PUT_LE16(buf, 0x00);                      //version
        PUT_LE16(buf, 0x20);                      //length of header in bytes
        PUT_LE32(buf, MKTAG('V', 'P', '8', '0')); //codec FourCC
        PUT_LE16(buf, avctx->width);                //width
        PUT_LE16(buf, avctx->height);               //height
        PUT_LE32(buf, avctx->framerate.num);      //frame rate
        PUT_LE32(buf, avctx->framerate.den);      //time scale(?)
        PUT_LE32(buf, avctx->frame_number);      //number of frames in file
        PUT_LE32(buf, 0); //unused
        size += 32;
        break;
    default:
        PUT_BUFFER(buf, extradata, extradata_size);
        size = extradata_size;
        break;
    }

    return size;
}

static int build_pic_header(unsigned char *buf, AVCodecContext *avctx, AVPacket *pkt, int bm_id, int *skip_size)
{
    int size = 0;
    unsigned char* pbchunk = pkt->data;
    //int has_start_code = 0;
    //int offset = 0;
    int fourcc = avctx->codec_tag;
    //int c_slice, n_slice;
    //int i, val;
    if (!fourcc)
        fourcc = codec_id_to_fourcc(avctx->codec_id);
    *skip_size = 0;
    switch(bm_id) {
    case STD_AVC:
        break;
    case STD_AVS:
        break;
    case STD_HEVC:
        break;
    case STD_VC1:
        if (!fourcc)
            return -1;

        if (fourcc == MKTAG('W', 'V', 'C', '1') || fourcc == MKTAG('W', 'M', 'V', 'A')) {
            if (pbchunk[0] != 0 || pbchunk[1] != 0 || pbchunk[2] != 1) {
                // check start code as prefix (0x00, 0x00, 0x01)
                buf[0] = 0x00;
                buf[1] = 0x00;
                buf[2] = 0x01;
                buf[3] = 0x0D; // replace to the correct picture header to indicate as frame

                size += 4;
            }
        } else {
            PUT_LE32(buf, pkt->size | ((pkt->flags & AV_PKT_FLAG_KEY) ? 0x80000000 : 0));
            size += 4;
#ifdef RCV_V2
            if (AV_NOPTS_VALUE == pkt->pts) {
                PUT_LE32(buf, 0);
            } else {
                PUT_LE32(buf, (int)((double)(pkt->pts/avctx->time_base.den))); // milli_sec
            }
            size += 4;
#endif
        }
        break;
    case STD_DIV3:
    case STD_VP8:
    case STD_VP9:
        PUT_LE32(buf,pkt->size);
        PUT_LE32(buf,0);
        PUT_LE32(buf,0);
        size += 12;
        break;
    }
    return size;
}

static int bm_decode_internal(AVCodecContext *avctx, void *outdata, int *outdata_size, AVPacket *avpkt)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    BMVidCodHandle handle = bmctx->handle;
    BMVidFrame *bmframe = (BMVidFrame *)malloc(sizeof(BMVidFrame));
    AVFrame *frame = (AVFrame *)outdata;
    BMVidStream stream;
    BMDecStatus dec_state;
    int ret = 0, get_frame_state = 0, flush_state = 0;
    int get_frame = 0, send_pkg = 0;
    int overtime_cnt = 0;
#ifdef BM_PCIE_MODE
    uint32_t align = 4;
    uint8_t *data_buf = NULL,*header_buf = NULL;
    uint32_t len = 0,len_aligned = 0,header_aligned = 0;
#endif
    *outdata_size = 0;

    if (bmctx->endof_flag == 0) {
        if(avpkt && avpkt->size>0) {
            unsigned char *tmp_buf = NULL;
            int tmp_offset = 0;
            int skip_size = 0;
            if(bmctx->extra_data_flag != 0) {
                if(bmctx->header_size != 0 && bmctx->header_size < avctx->extradata_size + 2048) {
                    bmctx->header_buf = realloc(bmctx->header_buf, avctx->extradata_size + 2048);
                    bmctx->header_size = avctx->extradata_size + 2048;
                }
                else if(bmctx->header_size == 0) {
                    bmctx->header_buf = malloc(avpkt->size + avctx->extradata_size + 2048);
                    bmctx->header_size = avctx->extradata_size + 2048;
                }
                tmp_buf = bmctx->header_buf;
                if(bmctx->pkg_num_inbuf==0) {
                    tmp_offset += build_seq_header(tmp_buf, avctx, avpkt, bmctx->bm_id);
                }
                tmp_offset += build_pic_header(tmp_buf+tmp_offset, avctx, avpkt, bmctx->bm_id, &skip_size);
                //memcpy(tmp_buf+tmp_offset, pkt.buf, pkt.size);
                stream.header_buf = tmp_buf;
#ifdef BM_PCIE_MODE
                /*header len aligned*/
                header_aligned = (tmp_offset + align - 1) & (~(align - 1));
                header_buf = (uint8_t *)av_mallocz(header_aligned);
                memcpy(header_buf,tmp_buf,tmp_offset);
                stream.header_buf = header_buf;
#endif
            }
#ifndef BM_PCIE_MODE
            stream.header_size = tmp_offset;
#else
            stream.header_size = header_aligned;
#endif
#ifdef BM_PCIE_MODE
            /*data len alined*/
            len = avpkt->size-skip_size;
            len_aligned = (len + align - 1) & (~(align - 1));
            data_buf = (uint8_t *)av_mallocz(len_aligned);
            memcpy(data_buf,avpkt->data + skip_size,avpkt->size - skip_size);
            stream.buf = data_buf;
            stream.length = len_aligned;
#else
            stream.buf = avpkt->data + skip_size;
            stream.length = avpkt->size - skip_size;
#endif
            stream.extradata_size = 0;
            stream.pts = avpkt->pts;
            stream.dts = avpkt->dts;

SEND_PKG:
            ret = bmvpu_dec_decode(handle, stream);
            if (ret == BM_SUCCESS) {
                send_pkg = 1;
                bmctx->pkg_num_inbuf += 1;
                bmctx->pkt_flag = 0;
                ret = avpkt->size;
#if 0
                av_log(avctx, AV_LOG_INFO,"pkg num: %d, empty size: %d, pkt size: %d, pts: %ld, dts: %ld\n",
                        bmctx->pkg_num_inbuf, bmvpu_dec_get_all_empty_input_buf_cnt(handle), stream.length, avpkt->pts, avpkt->dts);
#endif
                if (get_frame == 1) {
                    return ret;
                }
            } else {
                if(ret > 0)
                    ret = 0;

                if (get_frame == 1) {
                    usleep(100);
                    goto SEND_PKG;
                }
            }

#ifdef BM_PCIE_MODE
            //free the header and data buffer
            if(header_buf)
                av_freep(&header_buf);
            if(data_buf)
                av_freep(&data_buf);
#endif
        }
        else if(avpkt == NULL || avpkt->size==0) {
            bmctx->endof_flag = 1;
        }

    }
#if 0
    if (bmctx->pkg_num_inbuf==10 && bmctx->endof_flag == 0) {
        bmctx->endof_flag = 1; //for testing.....
    }
#endif

FLUSH_FRAME:
    if(/*bmvpu_dec_get_status(handle)<BMDEC_ENDOF && */bmctx->endof_flag==1) {
        // av_log(avctx, AV_LOG_INFO, "flush all frame in the decoder frame buffer\n");
        flush_state = bmvpu_dec_get_all_frame_in_buffer(handle);
        if(flush_state == 0)
            bmctx->endof_flag=2; //the command sent to vpu.
        else if(flush_state == BMDEC_FLUSH_BUF_FULL)   /* input buf full */
            bmctx->endof_flag=1;
    }

GET_FRAME:
    get_frame_state = bmvpu_dec_get_output(handle, bmframe);
    if(get_frame_state == BM_SUCCESS) {
        if(bmctx->first_frame_get == 0) {
            bmctx->first_frame_get = 1;
            int dts = bmframe->dts;
            if(dts < 0) {
                bmctx->pts_offset = 0 - dts;
            }
        }
        bmframe->pts = bmframe->dts + bmctx->pts_offset;
        overtime_cnt = 0;
        get_frame = 1;
    }
    else {
        if(bmvpu_dec_get_status(handle) == BMDEC_STOP && bmctx->endof_flag == 2) {
            bmctx->endof_flag = 0;
            bmctx->first_frame = 0;
            bmctx->first_frame_get = 0;
            bmctx->pkg_num_inbuf = 0;
            bmctx->pkt_flag = 0;

            free(bmframe);
            return AVERROR_EOF;
        }

        if(bmctx->endof_flag > 0) {
            usleep(1000);
            goto FLUSH_FRAME;
        }

        if(send_pkg == 1) {
            if(ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "maybe meet a error. didn't get frame.\n");
            }
            free(bmframe);
            return ret;
        }
        else {
            usleep(1000);
            overtime_cnt += 1;
            if(overtime_cnt == 10000){
                av_log(avctx, AV_LOG_ERROR, "maybe meet a error. didn't get frame. free input buffer:%d\n", bmvpu_dec_get_all_empty_input_buf_cnt(handle));
                overtime_cnt = 0;
                if(bmvpu_dec_get_status(handle) == BMDEC_STOP)
                    return AVERROR_EXTERNAL;
            }
            if (bmvpu_dec_get_all_empty_input_buf_cnt(handle) > 0)
                goto SEND_PKG;
            else
                goto GET_FRAME;
        }
    }

    if(bmctx->first_frame == 0 && avctx->skip_frame == AVDISCARD_ALL) {
        *outdata_size = 0;
        bmctx->first_frame = 1;
        if(avctx->width != bmframe->width ||
           avctx->height != bmframe->height ||
           bmctx->old_pix_fmt != bm_format_to_av_pixel_format(avctx, bmframe)) {
            BMVidStreamInfo streamInfo;

            avctx->width = bmframe->width;
            avctx->height = bmframe->height;
            avctx->coded_width = bmframe->coded_width;
            avctx->coded_height = bmframe->coded_height;
            bmctx->old_pix_fmt = bm_format_to_av_pixel_format(avctx, bmframe);
            if(bmvpu_dec_get_caps(bmctx->handle, &streamInfo)==0) {
                if(streamInfo.bitRate!=-1)
                    avctx->bit_rate = streamInfo.bitRate;
                avctx->level = streamInfo.level;
                avctx->profile = streamInfo.profile;
                avctx->has_b_frames = streamInfo.maxNumRefFrm;
                avctx->delay = streamInfo.frameBufDelay;
                if(streamInfo.fRateNumerator != -1 && streamInfo.fRateDenominator != -1) {
                    avctx->framerate.num = streamInfo.fRateNumerator;
                    avctx->framerate.den = streamInfo.fRateDenominator;
                }
                //avctx->frame_size = streamInfo.picWidth * streamInfo.picHeight;
                av_log(avctx, AV_LOG_INFO, "pic width: %d, height: %d, max_ref: %d\n",
                       avctx->width, avctx->height, avctx->has_b_frames);
                av_log(avctx, AV_LOG_INFO, "bitrate: %"PRId64", framerate num: %d, den: %d\n",
                       avctx->bit_rate, avctx->framerate.num, avctx->framerate.den);
            }
        }
    }

    if(bmctx->process_frame!=NULL && avctx->skip_frame != AVDISCARD_ALL) {
        bmctx->process_frame(bmctx->out_image_array, bmframe, bmctx->opque);
    }

    if(!(avctx->skip_frame != AVDISCARD_ALL && (bm_fill_frame(avctx, bmframe, frame) == 0))) {
        frame->flags = frame->flags | AV_FRAME_FLAG_DISCARD;
        frame->width = bmframe->width;
        frame->height = bmframe->height;
        frame->format = avctx->pix_fmt; // bm_format_to_av_pixel_format(avctx, bmframe);
        bmvpu_dec_clear_output(handle, bmframe);
        free(bmframe);
        bmframe = NULL;
    }

    if(avctx->get_buffer2 && bmctx->use_gst_flag){ // output frame is allocated by outside
        frame->reordered_opaque = avctx->reordered_opaque;
        avctx->get_buffer2(avctx, frame, 3);// To be determined, maybe not 3, now 264 software dec is 3
    }

    if (bmctx->perf) {
        struct timeval pe;
        gettimeofday(&pe, NULL);
        bmctx->total_time += ((pe.tv_sec*1000.0 + pe.tv_usec/1000.0) -
                          (bmctx->ps.tv_sec*1000.0 + bmctx->ps.tv_usec/1000.0));
    }

    //for testing
    //frame->flags = frame->flags | AV_FRAME_FLAG_DISCARD;
    *outdata_size = 1;
    if(bmctx->endof_flag == 0) {
        if(send_pkg == 0) {
            goto SEND_PKG;
        }
    }

    return ret;

}

static int bm_decode(AVCodecContext *avctx, void *outdata, int *outdata_size, AVPacket *avpkt)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);

    if (bmctx->perf) {
        gettimeofday(&bmctx->ps, NULL);
    }

    /* Flag not set, use default*/
    if (!bmctx->handle_packet_loss && bmctx->skip_non_idr==0) {
        if (!(avctx->flags2 & AV_CODEC_FLAG2_SHOW_ALL)) {
            if (bmctx->first_keyframe_coming == 0 && (avpkt->flags & AV_PKT_FLAG_KEY)) {
                bmctx->first_keyframe_coming = 1;
            }
            if (bmctx->first_keyframe_coming == 0) {
                av_log(avctx, AV_LOG_WARNING, "lost a non key frame in first serveral packet...\n");
                *outdata_size = 0;
                return 0;
            }
        }
        return bm_decode_internal(avctx, outdata, outdata_size, avpkt);
    }

    /* Only support H264/H265 for now*/
    if (avctx->codec_id != AV_CODEC_ID_H264 &&
        avctx->codec_id != AV_CODEC_ID_H265) {
        return bm_decode_internal(avctx, outdata, outdata_size, avpkt);
    }

    if (bmctx->skip_non_idr != 0) {
        if (avpkt->flags & AV_PKT_FLAG_KEY) {
            return bm_decode_internal(avctx, outdata, outdata_size, avpkt);
        }
        else {
            *outdata_size = 0;
            return 0;
        }
    }

    if (avpkt->flags & AV_PKT_FLAG_CORRUPT)  {
        bmctx->is_need_wait_iframe = 1;
        *outdata_size = 0;
        return 0;
    }

    if (bmctx->is_need_wait_iframe){
        if (avpkt->flags & AV_PKT_FLAG_KEY) {
            bmctx->is_need_wait_iframe = 0;
        }
    }

    if (bmctx->is_need_wait_iframe){
        *outdata_size = 0;
        return 0;
    }

    return bm_decode_internal(avctx, outdata, outdata_size, avpkt);
}

static av_cold void bm_flush_dec(AVCodecContext *avctx)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    BMVidCodHandle handle = bmctx->handle;

    bmvpu_dec_flush(handle);
}

static av_cold int bm_close(AVCodecContext *avctx)
{
    BMDecContext* bmctx = (BMDecContext*)(avctx->priv_data);
    BMVidCodHandle handle = bmctx->handle;
    bmctx->endof_flag = 100; //start_close
    if(bmctx->header_size>0 && bmctx->header_buf!=NULL)
        free(bmctx->header_buf);

    if (bmctx->perf) {
        if (bmctx->total_time > 0.0f) {
            av_log(avctx, AV_LOG_INFO, "Frames decoded: %ld. Decoding speed: %.1ffps. Time used: %.3fsec.\n",
                   bmctx->total_frame, bmctx->total_frame*1000/bmctx->total_time, bmctx->total_time/1000.0f);
        }
    }

#if defined(BM_PCIE_MODE)
    ff_mutex_lock(&tmp_av_frame.av_mutex);
    tmp_av_frame.inst_num -= 1;
    if (tmp_av_frame.inst_num == 0 && tmp_av_frame.data != NULL) {
        tmp_av_frame.size = 0;
        av_freep(&tmp_av_frame.data);
    }
    ff_mutex_unlock(&tmp_av_frame.av_mutex);
#endif

    return bm_dec_framebuffer_withlock_unref(&bmctx->bm_handle_buffer);
}

#define OFFSET(x) offsetof(BMDecContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

static const AVOption options[] = {
#if 0
    { "size_input_buffer", "size of input buffer in the bitmain decoder context",
        OFFSET(size_input_buffer), AV_OPT_TYPE_INT, {.i64 = 0x700000}, 0, INT_MAX, FLAGS },
    { "delay_min_buffers", "delay pkg num of input buffer in the bitmain decoder context",
        OFFSET(delay_min_buffers), AV_OPT_TYPE_INT, {.i64 = 4}, 1, INT_MAX, FLAGS },
#endif
    { "mode_bitstream", "mode of bit stream in the bitmain decoder context",
        OFFSET(mode_bitstream), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 2, FLAGS },
#if defined(BM1684) || defined(BM1686)
    { "output_format", "the output data format: uncompressed(0, at default) or compressed(101)",
        OFFSET(output_format), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS },
#else
    { "output_format", "the output data format: linear(0, at default) or tiled(100)",
        OFFSET(output_format), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS },
#endif
    { "cbcr_interleave", "cbcr interleave of output frame in the bitmain decoder context",
        OFFSET(cbcr_interleave), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 2, FLAGS },
    { "extra_frame_buffer_num", "number of extra frame buffer in the bitmain decoder context",
        OFFSET(extra_frame_buffer_num), AV_OPT_TYPE_INT, {.i64 = 7}, 0, INT_MAX, FLAGS },
    { "extra_data_flag", "process extra data flag of avpkt in the bitmain decoder context",
        OFFSET(extra_data_flag), AV_OPT_TYPE_INT, {.i64 = 1}, 0, INT_MAX, FLAGS },
    { "frame_delay", "return a display frame after frame_delay frames decoding in the bitmain decoder context",
        OFFSET(frame_delay), AV_OPT_TYPE_INT, {.i64 = -1}, INT_MIN, INT_MAX, FLAGS },
    { "pcie_board_id", "board id when running in pcie mode (DEPRECATED)",
        OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 256, FLAGS },
    { "pcie_no_copyback", "don't copy the decoded image back to CPU in pcie mode (DEPRECATED)",
        OFFSET(pcie_no_copyback), AV_OPT_TYPE_FLAGS, {.i64 = 1}, 0, 1, FLAGS },
    { "sophon_idx", "sophon device index when running in pcie mode",
        OFFSET(soc_idx), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 256, FLAGS },
    { "zero_copy", "don't copy the decoded image back to CPU in pcie mode",
        OFFSET(pcie_no_copyback), AV_OPT_TYPE_FLAGS, {.i64 = 1}, 0, 1, FLAGS },
    { "enable_cache", "enable output yuv frame buffer cache in the bitmain decoder context",
        OFFSET(enable_cache), AV_OPT_TYPE_INT, {.i64 = 0}, INT_MIN, INT_MAX, FLAGS },
    { "handle_packet_loss", "enable packet loss handling in the bitmain decoder context",
        OFFSET(handle_packet_loss), AV_OPT_TYPE_INT, {.i64 = 0}, INT_MIN, INT_MAX, FLAGS },
    { "skip_non_idr", "enable skip non idr frame in the bitmain decoder context",
        OFFSET(skip_non_idr), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 2, FLAGS },
    { "perf", "indicate if profile the performance", OFFSET(perf), AV_OPT_TYPE_FLAGS, {.i64 = 0}, 0, 1, FLAGS },
    { "secondary_axi", "enable second axi in the bitmain decoder. 0: disable, 1: enable, 2: auto",
        OFFSET(secondary_axi), AV_OPT_TYPE_INT, {.i64 = 2}, 0, 2, FLAGS },
    { "use_gst_flag", "When ffmpeg is called with GST",
        OFFSET(use_gst_flag), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, FLAGS },
    { "core_idx", "Specify a decoding core in a soc to decode. 1684 max = 3, 1684x max = 1",
        OFFSET(core_idx), AV_OPT_TYPE_INT, {.i64 = -1}, -1, 3, FLAGS },
    {"size_input_buffer","input buffer size of video decoder bitstream (0x200000, 0x700000)",
        OFFSET(size_input_buffer), AV_OPT_TYPE_INT, {.i64 = 0x400000} , 0X200000, 0X700000, FLAGS },
    {"cmd_queue","decoder command queue depth",
        OFFSET(dec_cmd_queue), AV_OPT_TYPE_INT, {.i64 = 4} , 1, 4, FLAGS },
    { NULL},
};

#if defined(BM1684) || defined(BM1686)
static const AVCodecHWConfigInternal *bmcodec_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_BMCODEC,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX | AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX, // TODO
            .device_type = AV_HWDEVICE_TYPE_BMCODEC,
        },
        .hwaccel = &ff_bmcodec_hwaccel,
    },
    NULL
};
#else
#define bmcodec_hw_configs NULL
#endif

#define BMDEC(NAME, LONGNAME, CODEC, bsf_name) \
static const AVClass bm_ ## NAME ## _dec_class = {\
    .class_name = #NAME "_bm_decoder",\
    .item_name  = av_default_item_name,\
    .option     = options,\
    .version    = LIBAVUTIL_VERSION_INT,\
};\
\
const FFCodec ff_ ## NAME ## _bm_decoder = { \
    .p.name           = #NAME "_bm" ,\
    CODEC_LONG_NAME("bm " LONGNAME " decoder wrapper"),\
    .p.type           = AVMEDIA_TYPE_VIDEO,\
    .p.id             = CODEC ,\
    .priv_data_size = sizeof(BMDecContext),\
    .p.priv_class     = &bm_ ## NAME ## _dec_class,\
    .init           = bm_decode_init,\
    FF_CODEC_DECODE_CB(bm_decode),\
    .close          = bm_close,\
    .flush          = bm_flush_dec,\
    .bsfs           = bsf_name, \
    .p.capabilities   = AV_CODEC_CAP_HARDWARE | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING, \
    .caps_internal  = FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM, \
    .hw_configs     = bmcodec_hw_configs, \
    .p.wrapper_name   = "bm", \
};


#if CONFIG_H264_BM_DECODER
BMDEC(h264,     "H.264",    AV_CODEC_ID_H264,       "h264_mp4toannexb");
#endif
#if CONFIG_HEVC_BM_DECODER
BMDEC(hevc,     "HEVC",     AV_CODEC_ID_HEVC,       "hevc_mp4toannexb");
#endif

#if CONFIG_VC1_BM_DECODER
BMDEC(vc1,      "VC1",      AV_CODEC_ID_VC1,        NULL);
#endif
#if CONFIG_WMV1_BM_DECODER
BMDEC(wmv1,     "WMV1",     AV_CODEC_ID_WMV1,       NULL);
#endif
#if CONFIG_WMV2_BM_DECODER
BMDEC(wmv2,     "WMV2",     AV_CODEC_ID_WMV2,       NULL);
#endif
#if CONFIG_WMV3_BM_DECODER
BMDEC(wmv3,     "WMV3",     AV_CODEC_ID_WMV3,       NULL);
#endif

#if CONFIG_MPEG1_BM_DECODER
BMDEC(mpeg1,    "MPEG1",    AV_CODEC_ID_MPEG1VIDEO, NULL);
#endif
#if CONFIG_MPEG2_BM_DECODER
BMDEC(mpeg2,    "MPEG2",    AV_CODEC_ID_MPEG2VIDEO, NULL);
#endif
#if CONFIG_MPEG4_BM_DECODER
BMDEC(mpeg4,    "MPEG4",    AV_CODEC_ID_MPEG4,      NULL);
#endif
#if CONFIG_MPEG4V3_BM_DECODER
BMDEC(mpeg4v3,  "MPEG4V3",  AV_CODEC_ID_MSMPEG4V3,  NULL);
#endif
#if CONFIG_FLV1_BM_DECODER
BMDEC(flv1,     "FLV1",     AV_CODEC_ID_FLV1,       NULL);
#endif

#if CONFIG_H263_BM_DECODER
BMDEC(h263,     "H.263",    AV_CODEC_ID_H263,       NULL);
#endif

#if CONFIG_CAVS_BM_DECODER
BMDEC(cavs,     "CAVS",     AV_CODEC_ID_CAVS,       NULL);
#endif
#if CONFIG_AVS_BM_DECODER
BMDEC(avs,      "AVS",      AV_CODEC_ID_AVS,        NULL);
#endif

#if CONFIG_VP3_BM_DECODER
BMDEC(vp3,      "VP3",      AV_CODEC_ID_VP3,        NULL);
#endif
#if CONFIG_VP8_BM_DECODER
BMDEC(vp8,      "VP8",      AV_CODEC_ID_VP8,        NULL);
#endif



