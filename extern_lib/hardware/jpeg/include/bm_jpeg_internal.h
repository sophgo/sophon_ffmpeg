#ifndef _BM_JPEG_INTERFACE_H_
#define _BM_JPEG_INTERFACE_H_

#include <stdint.h>
#include "bmlib_runtime.h"
#include "bm_jpeg_interface.h"

extern BmJpuLogLevel bm_jpu_cur_log_level_threshold;

void logging_fn(BmJpuLogLevel level, char const *file, int const line, char const *fn, const char *format, ...);

#define BM_JPU_ERROR_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_ERROR)   { logging_fn(BM_JPU_LOG_LEVEL_ERROR,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_WARNING_FULL(FILE_, LINE_, FUNCTION_, ...) do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_WARNING) { logging_fn(BM_JPU_LOG_LEVEL_WARNING, FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_INFO_FULL(FILE_, LINE_, FUNCTION_, ...)    do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_INFO)    { logging_fn(BM_JPU_LOG_LEVEL_INFO,    FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_DEBUG_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_DEBUG)   { logging_fn(BM_JPU_LOG_LEVEL_DEBUG,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_LOG_FULL(FILE_, LINE_, FUNCTION_, ...)     do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_LOG)     { logging_fn(BM_JPU_LOG_LEVEL_LOG,     FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_TRACE_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_TRACE)   { logging_fn(BM_JPU_LOG_LEVEL_TRACE,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)


#define BM_JPU_ERROR(...)    BM_JPU_ERROR_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_WARNING(...)  BM_JPU_WARNING_FULL(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_INFO(...)     BM_JPU_INFO_FULL   (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_DEBUG(...)    BM_JPU_DEBUG_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_LOG(...)      BM_JPU_LOG_FULL    (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_TRACE(...)    BM_JPU_TRACE_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)

typedef void* DecHandle;
typedef void* EncHandle;

typedef enum {
    FORMAT_420 = 0,
    FORMAT_422 = 1,
    FORMAT_224 = 2,
    FORMAT_444 = 3,
    FORMAT_400 = 4
} FrameFormat;

/* Packed Output Format */
typedef enum {
    PACKED_FORMAT_NONE,
    PACKED_FORMAT_422_YUYV,
    PACKED_FORMAT_422_UYVY,
    PACKED_FORMAT_422_YVYU,
    PACKED_FORMAT_422_VYUY,
    PACKED_FORMAT_444,
    PACKED_FORMAT_444_RGB
} PackedOutputFormat;

typedef enum
{
    FrameMode_Free,
    FrameMode_ContainsDisplayableFrame
} FrameMode;

typedef struct
{
    void *context;
    uint64_t pts;
    uint64_t dts;
    FrameMode mode;
} BmJpuDecFrameEntry;

typedef struct {
    uint64_t bufY;
    uint64_t bufCb;
    uint64_t bufCr;
    int strideY;
    int strideC;
    int myIndex;
} FrameBuffer;

typedef struct FramebufferList {
    struct FramebufferList *next;
    BmJpuFramebuffer *fb;
    int chn_id;
    int chn_fd;
} FramebufferList;

typedef struct {
    int picWidth;
    int picHeight;
    int minFrameBufferCount;
    int sourceFormat;
    int ecsPtr;
    int roiFrameWidth;
    int roiFrameHeight;
    int roiFrameOffsetX;
    int roiFrameOffsetY;
    int roiMCUSize;
    int colorComponents;
} DecInitialInfo;

typedef struct {
    int indexFrameDisplay;
    int numOfErrMBs;
    int decodingSuccess; /* 1: done; 0: error */
    int decPicHeight;
    int decPicWidth;
    int consumedByte;
    int bytePosFrameStart;
    int ecsPtr;
} DecOutputInfo;

/* Callback for handling new BmJpuDecInitialInfo data. This is called when new
 * information about the bitstream becomes available. output_code can be useful
 * to check why this callback was invoked. BM_JPU_DEC_OUTPUT_CODE_INITIAL_INFO_AVAILABLE
 * is always set. Every time this callback gets called, new framebuffers should be
 * allocated and registered with bm_jpu_dec_register_framebuffers().
 * user_data is a user-defined pointer that is passed to this callback. It has the same
 * value as the callback_user_data pointer from the bm_jpu_dec_open() call.
 * The callback returns 0 if something failed, nonzero if successful. */
DECL_EXPORT typedef int (*bm_jpu_dec_new_initial_info_callback)(BmJpuDecoder *decoder,
                                                    BmJpuDecInitialInfo *new_initial_info,
                                                    unsigned int output_code,
                                                    void *user_data);


/* Structure used together with bm_jpu_enc_open() */
typedef struct
{
    /* Width and height of the incoming frames, in pixels. These
     * do not have to be aligned to any boundaries. */
    unsigned int frame_width;
    unsigned int frame_height;
    /* Color format to use for incoming frames. MJPEG actually uses
     * all possible values.
     * See the BmJpuColorFormat documentation for an explanation how
     * the chroma_interleave value can affec the pixel format that is used. */
    BmJpuColorFormat color_format;

    /* Quality factor for JPEG encoding, between 1 (worst quality, best
     * compression) and 100 (best quality, worst compression). Default
     * value is 85.
     * This quality factor is the one from the Independent JPEG Group's
     * formula for generating a scale factor out of the quality factor.
     * This means that this quality factor is exactly the same as the
     * one used by libjpeg. */
    unsigned int quality_factor;

    /* If this is 1, then Cb and Cr are interleaved in one shared chroma
     * plane, otherwise they are separated in their own planes.
     * See the BmJpuColorFormat documentation for the consequences of this. */
    int chroma_interleave;

    int packed_format;
    int device_index;

    int rotationEnable;
    int mirrorEnable;
    int mirrorDirection;
    int rotationAngle;
} BmJpuEncOpenParams;

struct _BmJpuDecoder
{
    unsigned int device_index;
    DecHandle handle;

    bm_device_mem_t *bs_dma_buffer;
    uint8_t *bs_virt_addr;
    uint64_t bs_phys_addr;

    //unsigned int frame_width;
    //unsigned int frame_height;
    int chroma_interleave;
    int scale_ratio;

    unsigned int old_jpeg_width;
    unsigned int old_jpeg_height;
    BmJpuColorFormat old_jpeg_color_format;

    unsigned int num_framebuffers, num_used_framebuffers;
    /* internal_framebuffers and framebuffers are separate from
     * frame_entries: internal_framebuffers must be given directly
     * to the jpu_DecRegisterFrameBuffer() function, and framebuffers
     * is a user-supplied input value */
    FrameBuffer *internal_framebuffers;
    BmJpuFramebuffer *framebuffers;
    BmJpuDecFrameEntry *frame_entries;

    DecInitialInfo initial_info;
    int initial_info_available;

    DecOutputInfo dec_output_info;
    int available_decoded_frame_idx;

    bm_jpu_dec_new_initial_info_callback initial_info_callback;
    void *callback_user_data;

    int framebuffer_recycle;

    int channel_id;
    FramebufferList *fb_list_head;
    FramebufferList *fb_list_curr;

    int timeout;
};

struct _BmJpuEncoder
{
    unsigned int device_index;
    EncHandle handle;

    bm_device_mem_t *bs_dma_buffer;
    uint8_t *bs_virt_addr;

    BmJpuColorFormat color_format;
    unsigned int frame_width, frame_height;

    BmJpuFramebuffer *framebuffers;

    int channel_id;

    int timeout;
};

#endif
