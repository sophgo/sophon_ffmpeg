#ifndef _BM_JPEG_INTERFACE_H_
#define _BM_JPEG_INTERFACE_H_

#include <stdint.h>
#include "bmlib_runtime.h"


typedef enum {
    FORMAT_420 = 0,
    FORMAT_422 = 1,
    FORMAT_224 = 2,
    FORMAT_444 = 3,
    FORMAT_400 = 4
} FrameFormat;

/* Cb/Cr InterLeave */
typedef enum {
    CBCR_SEPARATED = 0,
    CBCR_INTERLEAVE,
    CRCB_INTERLEAVE
} CbCrInterLeave;

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
    /* planar 4:2:0; if the chroma_interleave parameter is 1, the corresponding format is NV12, otherwise it is I420 */
    BM_JPU_COLOR_FORMAT_YUV420            = 0,
    /* planar 4:2:2; if the chroma_interleave parameter is 1, the corresponding format is NV16 */
    BM_JPU_COLOR_FORMAT_YUV422_HORIZONTAL = 1,
    /* 4:2:2 vertical, actually 2:2:4 (according to the JPU docs); no corresponding format known for the chroma_interleave=1 case */
    /* NOTE: this format is rarely used, and has only been seen in a few JPEG files */
    BM_JPU_COLOR_FORMAT_YUV422_VERTICAL   = 2,
    /* planar 4:4:4; if the chroma_interleave parameter is 1, the corresponding format is NV24 */
    BM_JPU_COLOR_FORMAT_YUV444            = 3,
    /* 8-bit greayscale */
    BM_JPU_COLOR_FORMAT_YUV400            = 4,
    /* RGBP */
    BM_JPU_COLOR_FORMAT_RGB               = 5,
    /* BUTT */
    BM_JPU_COLOR_FORMAT_BUTT
} BmJpuColorFormat;


/* decode */
/* Decoder return codes. With the exception of BM_JPU_DEC_RETURN_CODE_OK, these
 * should be considered hard errors, and the decoder should be closed when they
 * are returned. */
typedef enum
{
    /* Operation finished successfully. */
    BM_JPU_DEC_RETURN_CODE_OK = 0,
    /* General return code for when an error occurs. This is used as a catch-all
     * for when the other error return codes do not match the error. */
    BM_JPU_DEC_RETURN_CODE_ERROR,
    /* Input parameters were invalid. */
    BM_JPU_DEC_RETURN_CODE_INVALID_PARAMS,
    /* JPU decoder handle is invalid. This is an internal error, and most likely
     * a bug in the library. Please report such errors. */
    BM_JPU_DEC_RETURN_CODE_INVALID_HANDLE,
    /* Framebuffer information is invalid. Typically happens when the BmJpuFramebuffer
     * structures that get passed to bm_jpu_dec_register_framebuffers() contain
     * invalid values. */
    BM_JPU_DEC_RETURN_CODE_INVALID_FRAMEBUFFER,
    /* Registering framebuffers for decoding failed because not enough framebuffers
     * were given to the bm_jpu_dec_register_framebuffers() function. */
    BM_JPU_DEC_RETURN_CODE_INSUFFICIENT_FRAMEBUFFERS,
    /* A stride value (for example one of the stride values of a framebuffer) is invalid. */
    BM_JPU_DEC_RETURN_CODE_INVALID_STRIDE,
    /* A function was called at an inappropriate time (for example, when
     * bm_jpu_dec_register_framebuffers() is called before a single byte of input data
     * was passed to bm_jpu_dec_decode() ). */
    BM_JPU_DEC_RETURN_CODE_WRONG_CALL_SEQUENCE,
    /* The operation timed out. */
    BM_JPU_DEC_RETURN_CODE_TIMEOUT,
    /* A function that should only be called once for the duration of the decoding
     * session was called again. One example is bm_jpu_dec_register_framebuffers(). */
    BM_JPU_DEC_RETURN_CODE_ALREADY_CALLED,
    /* Allocation memory failure */
    BM_JPU_DEC_RETURN_ALLOC_MEM_ERROR
} BmJpuDecReturnCodes;

typedef void* DecHandle;

typedef struct {
    uint64_t bufY;
    uint64_t bufCb;
    uint64_t bufCr;
    int strideY;
    int strideC;
    int myIndex;
} FrameBuffer;

/* Framebuffers are frame containers, and are used both for en- and decoding. */
typedef struct
{
    /* Stride of the Y and of the Cb&Cr components.
     * Specified in bytes. */
    unsigned int y_stride;
    unsigned int cbcr_stride;

    /* DMA buffer which contains the pixels. */
    bm_device_mem_t *dma_buffer;

    /* These define the starting offsets of each component
     * relative to the start of the buffer. Specified in bytes.
     */
    size_t y_offset;
    size_t cb_offset;
    size_t cr_offset;

    /* User-defined pointer. The library does not touch this value.
     * Not to be confused with the context fields of BmJpuEncodedFrame
     * and BmJpuRawFrame.
     * This can be used for example to identify which framebuffer out of
     * the initially allocated pool was used by the JPU to contain a frame.
     */
    void *context;

    /* Set to 1 if the framebuffer was already marked as displayed. This is for
     * internal use only. Not to be read or written from the outside. */
    int already_marked;

    /* Internal, implementation-defined data. Do not modify. */
    void *internal;
} BmJpuFramebuffer;

/* Frames are not just occupied or free. They can be in one of three modes:
 * - FrameMode_Free: framebuffer is not being used for decoding, and does not hold
     a displayable frame
 * - FrameMode_ContainsDisplayableFrame: framebuffer contains frame that has
 *   been fully decoded; this can be displayed
 *
 * Only frames in FrameMode_ContainsDisplayableFrame mode, via the
 * bm_jpu_dec_get_decoded_frame() function.
 */
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

typedef struct _BmJpuDecoder BmJpuDecoder;

/* Structure used together with bm_jpu_dec_new_initial_info_callback() .
 * The values are filled by the decoder. */
typedef struct
{
    /* Width of height of frames, in pixels. Note: it is not guaranteed that
     * these values are aligned to a 16-pixel boundary (which is required
     * for JPU framebuffers). These are the width and height of the frame
     * with actual pixel content. It may be a subset of the total frame,
     * in case these sizes need to be aligned. In that case, there are
     * padding columns to the right, and padding rows below the frames. */
    unsigned int frame_width, frame_height;

    /* Caller must register at least this many framebuffers
     * with the decoder. */
    unsigned int min_num_required_framebuffers;

    /* Color format of the decoded frames. */
    BmJpuColorFormat color_format;

    int chroma_interleave;

    /* Physical framebuffer addresses must be aligned to this value. */
    unsigned int framebuffer_alignment;

    int roiFrameWidth;
    int roiFrameHeight;
 } BmJpuDecInitialInfo;

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

typedef struct FramebufferList {
    struct FramebufferList *next;
    BmJpuFramebuffer *fb;
    int chn_id;
    int chn_fd;
} FramebufferList;

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
};

/* Structure used together with bm_jpu_calc_framebuffer_sizes() */
typedef struct
{
    /* Frame width and height, aligned to the 16-pixel boundary required by the JPU. */
    unsigned int aligned_frame_width, aligned_frame_height;

    /* Stride sizes, in bytes, with alignment applied. The Cb and Cr planes always
     * use the same stride, so they share the same value. */
    unsigned int y_stride, cbcr_stride;

    /* Required DMA memory size for the Y,Cb,Cr planes in bytes.
     * The Cb and Cr planes always are of the same size, so they share the same value. */
    unsigned int y_size, cbcr_size;

    /* Total required size of a framebuffer's DMA buffer, in bytes. This value includes
     * the sizes of all planes, and extra bytes for alignment and padding.
     * This value must be used when allocating DMA buffers for decoder framebuffers. */
    unsigned int total_size;

    /* This corresponds to the other chroma_interleave values used in bmjpuapi.
     * It is stored here to allow other functions to select the correct offsets. */
    int chroma_interleave;
} BmJpuFramebufferSizes;

/* Structure containing details about raw, uncompressed frames. */
typedef struct
{
    /* When decoding: pointer to the framebuffer containing the decoded raw frame.
     * When encoding: pointer to the framebuffer containing the raw frame to encode. */
    BmJpuFramebuffer *framebuffer;

    /* User-defined pointer. The library does not touch this value.
     * This pointer and the one from the corresponding encoded frame will have
     * the same value. The library will pass then through.
     * It can be used to identify which raw frame is associated with this
     * encoded frame for example. */
    void *context;

    /* User-defined timestamps. These are here for convenience. In many
     * cases, the context one wants to associate with raw/encoded frames
     * is a PTS-DTS pair. If only the context pointer were available, users
     * would have to create a separate data structure containing PTS & DTS
     * values for each context. Since this use case is common, these two
     * fields are added to the frame structure. Just like the context
     * pointer, the library just passes them through to the associated
     * encoded frame, and does not actually touch their values. It is also
     * perfectly OK to not use them, and just use the context pointer
     * instead, or vice versa. */
    uint64_t pts, dts;
} BmJpuRawFrame;

typedef struct _BMJpuJPEGDecoder
{
    BmJpuDecoder *decoder;

    bm_device_mem_t *bitstream_buffer;
    size_t bitstream_buffer_size;
    unsigned int bitstream_buffer_alignment;

    BmJpuDecInitialInfo initial_info;

    BmJpuFramebuffer *framebuffers;
    bm_device_mem_t *fb_dmabuffers;
    unsigned int num_framebuffers;
    unsigned int num_extra_framebuffers; // TODO
    BmJpuFramebufferSizes calculated_sizes;

    BmJpuRawFrame raw_frame;
    int device_index;

    BmJpuFramebuffer *cur_framebuffer;
    bm_device_mem_t *cur_dma_buffer;
    void *opaque;

    int rotationEnable;
    int mirrorEnable;
    int mirrorDirection;
    int rotationAngle;

    int framebuffer_recycle;
    size_t framebuffer_size;
} BmJpuJPEGDecoder;

/* Structure used together with bm_jpu_dec_open() */
typedef struct
{
    /* These are necessary with some formats which do not store the width
     * and height in the bitstream. If the format does store them, these
     * values can be set to zero. */
    unsigned int frame_width;
    unsigned int frame_height;

    BmJpuColorFormat color_format;
    /* If this is 1, then Cb and Cr are interleaved in one shared chroma
     * plane, otherwise they are separated in their own planes.
     * See the BmJpuColorFormat documentation for the consequences of this. */
    int chroma_interleave;

    /* 0: no scaling; n(1-3): scale by 2^n; */
    unsigned int scale_ratio;

    /* The DMA buffer size for bitstream */
    int bs_buffer_size;
#ifdef _WIN32
    uint8_t *buffer;
#else
    uint8_t *buffer __attribute__((deprecated));
#endif

    int device_index;

    int rotationEnable;
    int mirrorEnable;
    int mirrorDirection;
    int rotationAngle;

    int roiEnable;
    int roiWidth;
    int roiHeight;
    int roiOffsetX;
    int roiOffsetY;

    int framebuffer_recycle;
    size_t framebuffer_size;
} BmJpuDecOpenParams;

typedef struct
{
    /* Width and height of JPU framebuffers are aligned to internal boundaries.
     * The frame consists of the actual image pixels and extra padding pixels.
     * aligned_frame_width / aligned_frame_height specify the full width/height
     * including the padding pixels, and actual_frame_width / actual_frame_height
     * specify the width/height without padding pixels. */
    unsigned int aligned_frame_width, aligned_frame_height;
    unsigned int actual_frame_width, actual_frame_height;

    /* Stride and size of the Y, Cr, and Cb planes. The Cr and Cb planes always
     * have the same stride and size. */
    unsigned int y_stride, cbcr_stride;
    unsigned int y_size, cbcr_size;

    /* Offset from the start of a framebuffer's memory, in bytes. Note that the
     * Cb and Cr offset values are *not* the same, unlike the stride and size ones. */
    unsigned int y_offset, cb_offset, cr_offset;

    /* Framebuffer containing the pixels of the decoded frame. */
    BmJpuFramebuffer *framebuffer;

    /* Color format of the decoded frame. */
    BmJpuColorFormat color_format;

    int chroma_interleave;

    int framebuffer_recycle;
    size_t framebuffer_size;
} BmJpuJPEGDecInfo;


/* encode */
/* Encoder return codes. With the exception of BM_JPU_ENC_RETURN_CODE_OK, these
 * should be considered hard errors, and the encoder should be closed when they
 * are returned. */
typedef enum
{
    /* Operation finished successfully. */
    BM_JPU_ENC_RETURN_CODE_OK = 0,
    /* General return code for when an error occurs. This is used as a catch-all
     * for when the other error return codes do not match the error. */
    BM_JPU_ENC_RETURN_CODE_ERROR,
    /* Input parameters were invalid. */
    BM_JPU_ENC_RETURN_CODE_INVALID_PARAMS,
    /* JPU encoder handle is invalid. This is an internal error, and most likely
     * a bug in the library. Please report such errors. */
    BM_JPU_ENC_RETURN_CODE_INVALID_HANDLE,
    /* Framebuffer information is invalid. Typically happens when the BmJpuFramebuffer
     * structures that get passed to bm_jpu_enc_register_framebuffers() contain
     * invalid values. */
    BM_JPU_ENC_RETURN_CODE_INVALID_FRAMEBUFFER,
    /* Registering framebuffers for encoding failed because not enough framebuffers
     * were given to the bm_jpu_enc_register_framebuffers() function. */
    BM_JPU_ENC_RETURN_CODE_INSUFFICIENT_FRAMEBUFFERS,
    /* A stride value (for example one of the stride values of a framebuffer) is invalid. */
    BM_JPU_ENC_RETURN_CODE_INVALID_STRIDE,
    /* A function was called at an inappropriate time. */
    BM_JPU_ENC_RETURN_CODE_WRONG_CALL_SEQUENCE,
    /* The operation timed out. */
    BM_JPU_ENC_RETURN_CODE_TIMEOUT,
    /* write_output_data() in BmJpuEncParams returned 0. */
    BM_JPU_ENC_RETURN_CODE_WRITE_CALLBACK_FAILED,
    /* Allocation memory failure */
    BM_JPU_ENC_RETURN_ALLOC_MEM_ERROR
} BmJpuEncReturnCodes;

typedef void* EncHandle;

typedef struct _BmJpuEncoder
{
    unsigned int device_index;
    EncHandle handle;

    bm_device_mem_t *bs_dma_buffer;
    uint8_t *bs_virt_addr;

    BmJpuColorFormat color_format;
    unsigned int frame_width, frame_height;

    BmJpuFramebuffer *framebuffers;

    int channel_id;
} BmJpuEncoder;

/* Initial encoding information, produced by the encoder. This structure is
 * essential to actually begin encoding, since it contains all of the
 * necessary information to create and register enough framebuffers. */
typedef struct
{
    /* Caller must register at least this many framebuffers
     * with the encoder. */
    unsigned int min_num_required_framebuffers;

    /* Physical framebuffer addresses must be aligned to this value. */
    unsigned int framebuffer_alignment;
} BmJpuEncInitialInfo;

typedef struct _BmJpuJPEGEncoder
{
    BmJpuEncoder *encoder;

    bm_device_mem_t *bitstream_buffer;

    size_t bitstream_buffer_size;
    unsigned int bitstream_buffer_alignment;

    BmJpuEncInitialInfo initial_info;

    unsigned int frame_width, frame_height;

    BmJpuFramebufferSizes calculated_sizes;

    unsigned int quality_factor;

    BmJpuColorFormat color_format;
    int packed_format;
    int chroma_interleave;
    int device_index;

    int rotationEnable;
    int mirrorEnable;
    int mirrorDirection;
    int rotationAngle;
} BmJpuJPEGEncoder;

/* Structure containing details about encoded frames. */
typedef struct
{
    /* When decoding, data must point to the memory block which contains
     * encoded frame data that gets consumed by the JPU. Not used by
     * the encoder. */
    uint8_t *data;

    /* Size of the encoded data, in bytes. When decoding, this is set by
     * the user, and is the size of the encoded data that is pointed to
     * by data. When encoding, the encoder sets this to the size of the
     * acquired output block, in bytes (exactly the same value as the
     * acquire_output_buffer's size argument). */
    size_t data_size;

    /* Handle produced by the user-defined acquire_output_buffer function
     * during encoding. Not used by the decoder. */
    void *acquired_handle;

    /* User-defined pointer. The library does not touch this value.
     * This pointer and the one from the corresponding raw frame will have
     * the same value. The library will pass then through.
     * It can be used to identify which raw frame is associated with this
     * encoded frame for example. */
    void *context;

    /* User-defined timestamps. These are here for convenience. In many
     * cases, the context one wants to associate with raw/encoded frames
     * is a PTS-DTS pair. If only the context pointer were available, users
     * would have to create a separate data structure containing PTS & DTS
     * values for each context. Since this use case is common, these two
     * fields are added to the frame structure. Just like the context
     * pointer, the library just passes them through to the associated
     * raw frame, and does not actually touch their values. It is also
     * perfectly OK to not use them, and just use the context pointer
     * instead, or vice versa. */
    uint64_t pts, dts;
} BmJpuEncodedFrame;

/* Function pointer used during encoding for acquiring output buffers.
 * See bm_jpu_enc_encode() for details about the encoding process.
 * context is the value of output_buffer_context specified in
 * BmJpuEncParams. size is the size of the block to acquire, in bytes.
 * acquired_handle is an output value; the function can set this to a
 * handle that corresponds to the acquired buffer. For example, in
 * libav/FFmpeg, this handle could be a pointer to an AVBuffer. In
 * GStreamer, this could be a pointer to a GstBuffer. The value of
 * *acquired_handle will later be copied to the acquired_handle value
 * of BmJpuEncodedFrame.
 * The return value is a pointer to a memory-mapped region of the
 * output buffer, or NULL if acquiring failed.
 * If the write_output_data function pointer in the encoder params
 * is non-NULL, this function is not called.
 * This function is only used by bm_jpu_enc_encode(). */
typedef void* (*BmJpuEncAcquireOutputBuffer)(void *context, size_t size, void **acquired_handle);

/* Function pointer used during encoding for notifying that the encoder
 * is done with the output buffer. This is *not* a function for freeing
 * allocated buffers; instead, it makes it possible to release, unmap etc.
 * context is the value of output_buffer_context specified in
 * BmJpuEncParams. acquired_handle equals the value of *acquired_handle in
 * BmJpuEncAcquireOutputBuffer.
 * If the write_output_data function pointer in the encoder params
 * is non-NULL, this function is not called. */
typedef void (*BmJpuEncFinishOutputBuffer)(void *context, void *acquired_handle);

/* Function pointer used during encoding for passing the output encoded data
 * to the user. If this function is not NULL, then BmJpuEncFinishOutputBuffer
 * and BmJpuEncAcquireOutputBuffer function are not called. Instead, this
 * data write function is called whenever the library wants to write output.
 * encoded_frame contains valid pts, dts, and context data which was copied
 * over from the corresponding raw frame.
 * Returns 1 if writing succeeded, 0 otherwise.
 * */
typedef int (*BmJpuWriteOutputData)(void *context, uint8_t const *data, uint32_t size, BmJpuEncodedFrame *encoded_frame);

typedef struct
{
    /* Frame width and height of the input frame. These are the actual sizes;
     * they will be aligned internally if necessary. These sizes must not be
     * zero. */
    unsigned int frame_width, frame_height;

    /* Quality factor for JPEG encoding. 1 = best compression, 100 = best quality.
     * This is the exact same quality factor as used by libjpeg. */
    unsigned int quality_factor;

    /* Color format of the input frame. */
    BmJpuColorFormat color_format;

    /* Functions for acquiring and finishing output buffers. See the
     * typedef documentations in bmjpuapi.h for details about how
     * these functions should behave. */
    BmJpuEncAcquireOutputBuffer acquire_output_buffer;
    BmJpuEncFinishOutputBuffer finish_output_buffer;

    /* Function for directly passing the output data to the user
     * without copying it first.
     * Using this function will inhibit calls to acquire_output_buffer
     * and finish_output_buffer. */
    BmJpuWriteOutputData write_output_data;

    /* User supplied value that will be passed to the functions:
     * acquire_output_buffer, finish_output_buffer, write_output_data */
    void *output_buffer_context;

    int packed_format;
    int chroma_interleave;

    int rotationEnable;
    int mirrorEnable;
    int mirrorDirection;
    int rotationAngle;
} BmJpuJPEGEncParams;

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

/* export API */
#if defined (__cplusplus)
extern "C" {
#endif

int bm_jpu_calc_framebuffer_sizes(BmJpuColorFormat color_format, unsigned int frame_width, unsigned int frame_height, unsigned int framebuffer_alignment, int chroma_interleave, BmJpuFramebufferSizes *calculated_sizes);
void* bm_jpu_devm_map(uint64_t phys_addr, size_t len);
void bm_jpu_devm_unmap(void *virt_addr, size_t len);

int bm_jpu_dec_get_channel_fd(int chn_id);
bm_handle_t bm_jpu_dec_get_bm_handle(int device_index);
BmJpuDecReturnCodes bm_jpu_dec_load(int device_index);
BmJpuDecReturnCodes bm_jpu_dec_unload(int device_index);
BmJpuDecReturnCodes bm_jpu_jpeg_dec_open(BmJpuJPEGDecoder **jpeg_decoder, BmJpuDecOpenParams *open_params, unsigned int num_extra_framebuffers);
BmJpuDecReturnCodes bm_jpu_jpeg_dec_close(BmJpuJPEGDecoder *jpeg_decoder);
BmJpuDecReturnCodes bm_jpu_jpeg_dec_decode(BmJpuJPEGDecoder *jpeg_decoder, uint8_t const *jpeg_data, size_t const jpeg_data_size);
void bm_jpu_jpeg_dec_get_info(BmJpuJPEGDecoder *jpeg_decoder, BmJpuJPEGDecInfo *info);
BmJpuDecReturnCodes bm_jpu_jpeg_dec_frame_finished(BmJpuJPEGDecoder *jpeg_decoder, BmJpuFramebuffer *framebuffer);
BmJpuDecReturnCodes bm_jpu_jpeg_dec_flush(BmJpuJPEGDecoder *jpeg_decoder);

int bm_jpu_enc_get_channel_fd(int chn_id);
bm_handle_t bm_jpu_enc_get_bm_handle(int device_index);
BmJpuEncReturnCodes bm_jpu_enc_load(int device_index);
BmJpuEncReturnCodes bm_jpu_enc_unload(int device_index);
BmJpuEncReturnCodes bm_jpu_jpeg_enc_open(BmJpuJPEGEncoder **jpeg_encoder, int bs_buffer_size, int device_index);
BmJpuEncReturnCodes bm_jpu_jpeg_enc_close(BmJpuJPEGEncoder *jpeg_encoder);
BmJpuEncReturnCodes bm_jpu_jpeg_enc_encode(BmJpuJPEGEncoder *jpeg_encoder, BmJpuFramebuffer const *framebuffer, BmJpuJPEGEncParams const *params, void **acquired_handle, size_t *output_buffer_size);

#if defined (__cplusplus)
}
#endif

#endif /* _BM_JPEG_INTERFACE_H_ */