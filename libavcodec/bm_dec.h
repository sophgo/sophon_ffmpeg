#ifndef _BM_DEC_H_
#define _BM_DEC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <libavformat/avformat.h>
#include <libavutil/avutil.h>

/**
* @brief    This is an enumeration type for representing picture types.
*/
typedef enum {
    BM_PIC_TYPE_I            = 0, /**< I picture */
    BM_PIC_TYPE_KEY          = 0, /**< KEY frame for SVAC */
    BM_PIC_TYPE_P            = 1, /**< P picture */
    BM_PIC_TYPE_INTER        = 1, /**< Inter frame for SVAC */
    BM_PIC_TYPE_B            = 2, /**< B picture (except VC1) */
    BM_PIC_TYPE_REPEAT       = 2, /**< Repeat frame (VP9 only) */
    BM_PIC_TYPE_VC1_BI       = 2, /**< VC1 BI picture (VC1 only) */
    BM_PIC_TYPE_VC1_B        = 3, /**< VC1 B picture (VC1 only) */
    BM_PIC_TYPE_D            = 3, /**< D picture in MPEG2 that is only composed of DC coefficients (MPEG2 only) */
    BM_PIC_TYPE_S            = 3, /**< S picture in MPEG4 that is an acronym of Sprite and used for GMC (MPEG4 only)*/
    BM_PIC_TYPE_AVS2_F       = 3, /**< F picture in AVS2 */
    BM_PIC_TYPE_VC1_P_SKIP   = 4, /**< VC1 P skip picture (VC1 only) */
    BM_PIC_TYPE_MP4_P_SKIP_NOT_CODED = 4, /**< Not Coded P Picture in MPEG4 packed mode */
    BM_PIC_TYPE_AVS2_S       = 4, /**< S picture in AVS2 */
    BM_PIC_TYPE_IDR          = 5, /**< H.264/H.265 IDR picture */
    BM_PIC_TYPE_AVS2_G       = 5, /**< G picture in AVS2 */
    BM_PIC_TYPE_AVS2_GB      = 6, /**< GB picture in AVS2 */
    BM_PIC_TYPE_MAX               /**< No Meaning */
} PicType;

typedef struct{
    void          *handle;
    int           ref_count;
    AVMutex       av_mutex;
}BMHandleBuffer;

#ifndef BMVIDFRAME
#define BMVIDFRAME
typedef struct _BMVidFrame {
    int picType;
    unsigned char* buf[8]; /**< 0: Y virt addr, 1: Cb virt addr: 2, Cr virt addr. 4: Y phy addr, 5: Cb phy addr, 6: Cr phy addr */
    int stride[8];
    unsigned int width;
    unsigned int height;
    int frameFormat;
    int interlacedFrame;
    int lumaBitDepth;   /**< Bit depth for luma component */
    int chromaBitDepth; /**< Bit depth for chroma component  */
    int cbcrInterleave;
    int nv21;
    int endian;
    int sequenceNo;  /**< This variable increases by 1 whenever sequence changes (WAVE only) */
    int frameIdx;
    unsigned long pts;
    unsigned long dts;
    int colorPrimaries;
    int colorTransferCharacteristic;
    int colorSpace;
    int colorRange;
    int chromaLocation;
    int size; /**< Framebuffer size */
    unsigned int coded_width;
    unsigned int coded_height;
} BMVidFrame;
#endif

typedef struct _BMDecContext{
    const AVClass *avclass;
    AVCodecContext *avctx;
    void *handle;
    int endof_flag;
    int start_flag;
    int pkg_num_inbuf;
    int (*process_frame)(void *out_image_array, BMVidFrame *surface_data, int64_t opque);
    void *out_image_array;
    int64_t opque;
    int first_frame;
    int bm_id;
    int mp4class;
    unsigned char *header_buf;
    int header_size;
    int size_input_buffer;
    int mode_bitstream;//0 interrupt 2 pic_end
    int delay_min_buffers;
    AVPacket pkt;
    int pkt_flag;
    int output_format;
    int cbcr_interleave;
    int extra_frame_buffer_num;
    int extra_data_flag;
    int frame_delay;
    int soc_idx;
    int pcie_no_copyback;
    int enable_cache;
    int handle_packet_loss;
    int is_need_wait_iframe;
    int skip_non_idr;

    int    hw_accel;

    enum AVPixelFormat old_pix_fmt;

    int    perf;        /* indicate if do the performance testing */
    struct timeval ps;
    double total_time;  /* ms */
    long   total_frame;

    int secondary_axi;
    int first_keyframe_coming;
    BMHandleBuffer  *bm_handle_buffer;
    int reserved[11];
    int use_gst_flag;
    int core_idx;
    int first_frame_get;
    int pts_offset;
    int dec_cmd_queue;
} BMDecContext;

#ifdef __cplusplus
}
#endif

#endif /* _BM_DEC_H_ */
