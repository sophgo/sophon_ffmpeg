/*
 * H.264 encoding using the bmx264 hardware acceleration
 *
 * Copyright (C) 2020 Solan Shang <shulin.shang@bitmain.com>
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

#ifndef __BMX264_COMMON_H__
#define __BMX264_COMMON_H__

#if defined(BM_PCIE_MODE) && defined(BM1684)
#include <stdint.h>
#include "libavcodec/avcodec.h"
#include "bmlib_runtime.h"

enum {
    BMCPU_LOGLEVEL_DEBUG  = 0,
    BMCPU_LOGLEVEL_INFO   = 1,
    BMCPU_LOGLEVEL_WARN   = 2,
    BMCPU_LOGLEVEL_ERROR  = 3,
    BMCPU_LOGLEVEL_FATAL  = 4,
    BMCPU_LOGLEVEL_NONE   = 5,
};

enum {
    LOG_TO_CONSOLE_OFF = 0,
    LOG_TO_CONSOLE_ON  = 1,
};

/* Threading TODO */
#define X264_THREADS_AUTO 0           /* Automatically select optimal number of threads */
#define X264_SYNC_LOOKAHEAD_AUTO (-1) /* Automatically select optimal lookahead thread buffer size */

/* Log level */
enum {
    X264_LOG_NONE    = -1,
    X264_LOG_ERROR   =  0,
    X264_LOG_WARNING =  1,
    X264_LOG_INFO    =  2,
    X264_LOG_DEBUG   =  3,
};

/* Rate control */
enum {
    X264_RC_CQP = 0,
    X264_RC_CRF = 1,
    X264_RC_ABR = 2
};

/* Slice type */
enum {
    X264_TYPE_AUTO     = 0,  /* Let x264 choose the right type */
    X264_TYPE_IDR      = 1,
    X264_TYPE_I        = 2,
    X264_TYPE_P        = 3,
    X264_TYPE_BREF     = 4,  /* Non-disposable B-frame */
    X264_TYPE_B        = 5,
    X264_TYPE_KEYFRAME = 6,  /* IDR or I depending on b_open_gop option */
};

#define IS_X264_TYPE_I(x) ((x)==X264_TYPE_I || (x)==X264_TYPE_IDR || (x)==X264_TYPE_KEYFRAME)
#define IS_X264_TYPE_B(x) ((x)==X264_TYPE_B || (x)==X264_TYPE_BREF)

/* Colorspace type */
enum {
    X264_CSP_MASK       = 0x00ff,  /* */
    X264_CSP_NONE       = 0x0000,  /* Invalid mode     */
    X264_CSP_I400       = 0x0001,  /* monochrome 4:0:0 */
    X264_CSP_I420       = 0x0002,  /* yuv 4:2:0 planar */
    X264_CSP_YV12       = 0x0003,  /* yvu 4:2:0 planar */
    X264_CSP_NV12       = 0x0004,  /* yuv 4:2:0, with one y plane and one packed u+v */
    X264_CSP_NV21       = 0x0005,  /* yuv 4:2:0, with one y plane and one packed v+u */
    X264_CSP_I422       = 0x0006,  /* yuv 4:2:2 planar */
    X264_CSP_YV16       = 0x0007,  /* yvu 4:2:2 planar */
    X264_CSP_NV16       = 0x0008,  /* yuv 4:2:2, with one y plane and one packed u+v */
    X264_CSP_YUYV       = 0x0009,  /* yuyv 4:2:2 packed */
    X264_CSP_UYVY       = 0x000a,  /* uyvy 4:2:2 packed */
    X264_CSP_V210       = 0x000b,  /* 10-bit yuv 4:2:2 packed in 32 */
    X264_CSP_I444       = 0x000c,  /* yuv 4:4:4 planar */
    X264_CSP_YV24       = 0x000d,  /* yvu 4:4:4 planar */
    X264_CSP_BGR        = 0x000e,  /* packed bgr 24bits */
    X264_CSP_BGRA       = 0x000f,  /* packed bgr 32bits */
    X264_CSP_RGB        = 0x0010,  /* packed rgb 24bits */
    X264_CSP_MAX        = 0x0011,  /* end of list */
    X264_CSP_VFLIP      = 0x1000,  /* the csp is vertically flipped */
    X264_CSP_HIGH_DEPTH = 0x2000,  /* the csp has a depth of 16 bits per pixel component */
};

/* Analyse flags TODO */
enum {
    X264_ANALYSE_I4x4       = 0x0001U,  /* Analyse i4x4 */
    X264_ANALYSE_I8x8       = 0x0002U,  /* Analyse i8x8 (requires 8x8 transform) */
    X264_ANALYSE_PSUB16x16  = 0x0010U,  /* Analyse p16x8, p8x16 and p8x8 */
    X264_ANALYSE_PSUB8x8    = 0x0020U,  /* Analyse p8x4, p4x8, p4x4 */
    X264_ANALYSE_BSUB16x16  = 0x0100U,  /* Analyse b16x8, b8x16 and b8x8 */
};

/* TODO */
enum {
    X264_CQM_FLAT   = 0,
    X264_CQM_JVT    = 1,
    X264_CQM_CUSTOM = 2,
};

enum {
    NAL_UNKNOWN     = 0,
    NAL_SLICE       = 1,
    NAL_SLICE_DPA   = 2,
    NAL_SLICE_DPB   = 3,
    NAL_SLICE_DPC   = 4,
    NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
    NAL_SEI         = 6,    /* ref_idc == 0 */
    NAL_SPS         = 7,
    NAL_PPS         = 8,
    NAL_AUD         = 9,
    NAL_FILLER      = 12,
    /* ref_idc == 0 for 6,9,10,11,12 */
};

typedef struct x264_t x264_t;

typedef struct x264_zone_t {
    int i_start, i_end; /* range of frame numbers */
    int b_force_qp; /* whether to use qp vs bitrate factor */
    int i_qp;
    float f_bitrate_factor;
    struct x264_param_t *param;
} x264_zone_t;

typedef struct x264_hrd_t {
    double cpb_initial_arrival_time;
    double cpb_final_arrival_time;
    double cpb_removal_time;

    double dpb_output_time;
} x264_hrd_t;

typedef struct x264_sei_payload_t {
    int payload_size;
    int payload_type;
    uint8_t *payload;
} x264_sei_payload_t;

typedef struct x264_sei_t {
    int num_payloads;
    x264_sei_payload_t *payloads;
    /* In: optional callback to free each payload AND x264_sei_payload_t when used. */
    void (*sei_free)( void* );
} x264_sei_t;

/* The data within the payload is already NAL-encapsulated; the ref_idc and type
 * are merely in the struct for easy access by the calling application.
 * All data returned in an x264_nal_t, including the data in p_payload, is no longer
 * valid after the next call to x264_encoder_encode.  Thus it must be used or copied
 * before calling x264_encoder_encode or x264_encoder_headers again. */
typedef struct x264_nal_t {
    int i_ref_idc;  /* nal_priority_e */
    int i_type;     /* nal_unit_type_e */
    int b_long_startcode;
    int i_first_mb; /* If this NAL is a slice, the index of the first MB in the slice. */
    int i_last_mb;  /* If this NAL is a slice, the index of the last MB in the slice. */

    /* Size of payload (including any padding) in bytes. */
    int     i_payload;
    /* If param->b_annexb is set, Annex-B bytestream with startcode.
     * Otherwise, startcode is replaced with a 4-byte size.
     * This size is the size used in mp4/similar muxing; it is equal to i_payload-4 */
    uint8_t *p_payload;

    /* Size of padding in bytes. */
    int i_padding;
} x264_nal_t;

typedef struct x264_param_t {
    /* CPU flags */
    uint32_t    cpu;
    int         i_threads;           /* encode multiple frames in parallel */
    int         i_lookahead_threads; /* multiple threads for lookahead analysis */
    int         b_sliced_threads;  /* Whether to use slice-based threading. */
    int         b_deterministic; /* whether to allow non-deterministic optimizations when threaded */
    int         b_cpu_independent; /* force canonical behavior rather than cpu-dependent optimal algorithms */
    int         i_sync_lookahead; /* threaded lookahead buffer */

    /* Video Properties */
    int         i_width;
    int         i_height;
    int         i_csp;         /* CSP of encoded bitstream */
    int         i_bitdepth;
    int         i_level_idc;
    int         i_frame_total; /* number of frames to encode if known, else 0 */

    /* NAL HRD
     * Uses Buffering and Picture Timing SEIs to signal HRD
     * The HRD in H.264 was not designed with VFR in mind.
     * It is therefore not recommendeded to use NAL HRD with VFR.
     * Furthermore, reconfiguring the VBV (via x264_encoder_reconfig)
     * will currently generate invalid HRD. */
    int         i_nal_hrd;

    struct {
        /* they will be reduced to be 0 < x <= 65535 and prime */
        int         i_sar_height;
        int         i_sar_width;

        int         i_overscan;    /* 0=undef, 1=no overscan, 2=overscan */

        /* see h264 annex E for the values of the following */
        int         i_vidformat;
        int         b_fullrange;
        int         i_colorprim;
        int         i_transfer;
        int         i_colmatrix;
        int         i_chroma_loc;    /* both top & bottom */
    } vui;

    /* Bitstream parameters */
    int         i_frame_reference;  /* Maximum number of reference frames */
    int         i_dpb_size;         /* Force a DPB size larger than that implied by B-frames and reference frames.
                                     * Useful in combination with interactive error resilience. */
    int         i_keyint_max;       /* Force an IDR keyframe at this interval */
    int         i_keyint_min;       /* Scenecuts closer together than this are coded as I, not IDR. */
    int         i_scenecut_threshold; /* how aggressively to insert extra I frames */
    int         b_intra_refresh;    /* Whether or not to use periodic intra refresh instead of IDR frames. */

    int         i_bframe;   /* how many b-frame between 2 references pictures */
    int         i_bframe_adaptive;
    int         i_bframe_bias;
    int         i_bframe_pyramid;   /* Keep some B-frames as references: 0=off, 1=strict hierarchical, 2=normal */
    int         b_open_gop;
    int         b_bluray_compat;
    int         i_avcintra_class;
    int         i_avcintra_flavor;

    int         b_deblocking_filter;
    int         i_deblocking_filter_alphac0;    /* [-6, 6] -6 light filter, 6 strong */
    int         i_deblocking_filter_beta;       /* [-6, 6]  idem */

    int         b_cabac;
    int         i_cabac_init_idc;

    int         b_interlaced;
    int         b_constrained_intra;

    int         i_cqm_preset;
    char        *psz_cqm_file;      /* filename (in UTF-8) of CQM file, JM format */
    uint8_t     cqm_4iy[16];        /* used only if i_cqm_preset == X264_CQM_CUSTOM */
    uint8_t     cqm_4py[16];
    uint8_t     cqm_4ic[16];
    uint8_t     cqm_4pc[16];
    uint8_t     cqm_8iy[64];
    uint8_t     cqm_8py[64];
    uint8_t     cqm_8ic[64];
    uint8_t     cqm_8pc[64];

    /* Log */
    void        (*pf_log)( void *, int i_level, const char *psz, va_list );
    void        *p_log_private;
    int         i_log_level;
    int         b_full_recon;   /* fully reconstruct frames, even when not necessary for encoding.  Implied by psz_dump_yuv */
    char        *psz_dump_yuv;  /* filename (in UTF-8) for reconstructed frames */

    /* Encoder analyser parameters */
    struct {
        unsigned int intra;     /* intra partitions */
        unsigned int inter;     /* inter partitions */

        int          b_transform_8x8;
        int          i_weighted_pred; /* weighting for P-frames */
        int          b_weighted_bipred; /* implicit weighting for B-frames */
        int          i_direct_mv_pred; /* spatial vs temporal mv prediction */
        int          i_chroma_qp_offset;

        int          i_me_method; /* motion estimation algorithm to use (X264_ME_*) */
        int          i_me_range; /* integer pixel motion estimation search range (from predicted mv) */
        int          i_mv_range; /* maximum length of a mv (in pixels). -1 = auto, based on level */
        int          i_mv_range_thread; /* minimum space between threads. -1 = auto, based on number of threads. */
        int          i_subpel_refine; /* subpixel motion estimation quality */
        int          b_chroma_me; /* chroma ME for subpel and mode decision in P-frames */
        int          b_mixed_references; /* allow each mb partition to have its own reference number */
        int          i_trellis;  /* trellis RD quantization */
        int          b_fast_pskip; /* early SKIP detection on P-frames */
        int          b_dct_decimate; /* transform coefficient thresholding on P-frames */
        int          i_noise_reduction; /* adaptive pseudo-deadzone */
        float        f_psy_rd; /* Psy RD strength */
        float        f_psy_trellis; /* Psy trellis strength */
        int          b_psy; /* Toggle all psy optimizations */

        int          b_mb_info;            /* Use input mb_info data in x264_picture_t */
        int          b_mb_info_update; /* Update the values in mb_info according to the results of encoding. */

        /* the deadzone size that will be used in luma quantization */
        int          i_luma_deadzone[2]; /* {inter, intra} */

        int          b_psnr;    /* compute and print PSNR stats */
        int          b_ssim;    /* compute and print SSIM stats */
    } analyse;

    /* Rate control parameters */
    struct {
        int         i_rc_method;    /* X264_RC_* */

        int         i_qp_constant;  /* 0=lossless */
        int         i_qp_min;       /* min allowed QP value */
        int         i_qp_max;       /* max allowed QP value */
        int         i_qp_step;      /* max QP step between frames */

        int         i_bitrate;
        float       f_rf_constant;  /* 1pass VBR, nominal QP */
        float       f_rf_constant_max;  /* In CRF mode, maximum CRF as caused by VBV */
        float       f_rate_tolerance;
        int         i_vbv_max_bitrate;
        int         i_vbv_buffer_size;
        float       f_vbv_buffer_init; /* <=1: fraction of buffer_size. >1: kbit */
        float       f_ip_factor;
        float       f_pb_factor;

        /* VBV filler: force CBR VBV and use filler bytes to ensure hard-CBR.
         * Implied by NAL-HRD CBR. */
        int         b_filler;

        int         i_aq_mode;      /* psy adaptive QP. (X264_AQ_*) */
        float       f_aq_strength;
        int         b_mb_tree;      /* Macroblock-tree ratecontrol. */
        int         i_lookahead;

        /* 2pass */
        int         b_stat_write;   /* Enable stat writing in psz_stat_out */
        char        *psz_stat_out;  /* output filename (in UTF-8) of the 2pass stats file */
        int         b_stat_read;    /* Read stat from psz_stat_in and use it */
        char        *psz_stat_in;   /* input filename (in UTF-8) of the 2pass stats file */

        /* 2pass params (same as ffmpeg ones) */
        float       f_qcompress;    /* 0.0 => cbr, 1.0 => constant qp */
        float       f_qblur;        /* temporally blur quants */
        float       f_complexity_blur; /* temporally blur complexity */
        x264_zone_t *zones;         /* ratecontrol overrides */
        int         i_zones;        /* number of zone_t's */
        char        *psz_zones;     /* alternate method of specifying zones */
    } rc;

    /* Cropping Rectangle parameters: added to those implicitly defined by
       non-mod16 video resolutions. */
    struct {
        int i_left;
        int i_top;
        int i_right;
        int i_bottom;
    } crop_rect;

    /* frame packing arrangement flag */
    int i_frame_packing;

    /* alternative transfer SEI */
    int i_alternative_transfer;

    /* Muxing parameters */
    int b_aud;                  /* generate access unit delimiters */
    int b_repeat_headers;       /* put SPS/PPS before each keyframe */
    int b_annexb;               /* if set, place start codes (4 bytes) before NAL units,
                                 * otherwise place size (4 bytes) before NAL units. */
    int i_sps_id;               /* SPS and PPS id number */
    int b_vfr_input;            /* VFR input.  If 1, use timebase and timestamps for ratecontrol purposes.
                                 * If 0, use fps only. */
    int b_pulldown;             /* use explicity set timebase for CFR */
    uint32_t i_fps_num;
    uint32_t i_fps_den;
    uint32_t i_timebase_num;    /* Timebase numerator */
    uint32_t i_timebase_den;    /* Timebase denominator */

    int b_tff;

    /* Pulldown:
     * The correct pic_struct must be passed with each input frame.
     * The input timebase should be the timebase corresponding to the output framerate. This should be constant.
     * e.g. for 3:2 pulldown timebase should be 1001/30000
     * The PTS passed with each frame must be the PTS of the frame after pulldown is applied.
     * Frame doubling and tripling require b_vfr_input set to zero (see H.264 Table D-1)
     *
     * Pulldown changes are not clearly defined in H.264. Therefore, it is the calling app's responsibility to manage this.
     */

    int b_pic_struct;

    /* Fake Interlaced.
     *
     * Used only when b_interlaced=0. Setting this flag makes it possible to flag the stream as PAFF interlaced yet
     * encode all frames progessively. It is useful for encoding 25p and 30p Blu-Ray streams.
     */

    int b_fake_interlaced;

    /* Don't optimize header parameters based on video content, e.g. ensure that splitting an input video, compressing
     * each part, and stitching them back together will result in identical SPS/PPS. This is necessary for stitching
     * with container formats that don't allow multiple SPS/PPS. */
    int b_stitchable;

    int b_opencl;            /* use OpenCL when available */
    int i_opencl_device;     /* specify count of GPU devices to skip, for CLI users */
    void *opencl_device_id;  /* pass explicit cl_device_id as void*, for API users */
    char *psz_clbin_file;    /* filename (in UTF-8) of the compiled OpenCL kernel cache file */

    /* Slicing parameters */
    int i_slice_max_size;    /* Max size per slice in bytes; includes estimated NAL overhead. */
    int i_slice_max_mbs;     /* Max number of MBs per slice; overrides i_slice_count. */
    int i_slice_min_mbs;     /* Min number of MBs per slice */
    int i_slice_count;       /* Number of slices per frame: forces rectangular slices. */
    int i_slice_count_max;   /* Absolute cap on slices per frame; stops applying slice-max-size
                              * and slice-max-mbs if this is reached. */

    /* Optional callback for freeing this x264_param_t when it is done being used.
     * Only used when the x264_param_t sits in memory for an indefinite period of time,
     * i.e. when an x264_param_t is passed to x264_t in an x264_picture_t or in zones.
     * Not used when x264_encoder_reconfig is called directly. */
    void (*param_free)( void* );

    /* Optional low-level callback for low-latency encoding.  Called for each output NAL unit
     * immediately after the NAL unit is finished encoding.  This allows the calling application
     * to begin processing video data (e.g. by sending packets over a network) before the frame
     * is done encoding.
     *
     * This callback MUST do the following in order to work correctly:
     * 1) Have available an output buffer of at least size nal->i_payload*3/2 + 5 + 64.
     * 2) Call x264_nal_encode( h, dst, nal ), where dst is the output buffer.
     * After these steps, the content of nal is valid and can be used in the same way as if
     * the NAL unit were output by x264_encoder_encode.
     *
     * This does not need to be synchronous with the encoding process: the data pointed to
     * by nal (both before and after x264_nal_encode) will remain valid until the next
     * x264_encoder_encode call.  The callback must be re-entrant.
     *
     * This callback does not work with frame-based threads; threads must be disabled
     * or sliced-threads enabled.  This callback also does not work as one would expect
     * with HRD -- since the buffering period SEI cannot be calculated until the frame
     * is finished encoding, it will not be sent via this callback.
     *
     * Note also that the NALs are not necessarily returned in order when sliced threads is
     * enabled.  Accordingly, the variable i_first_mb and i_last_mb are available in
     * x264_nal_t to help the calling application reorder the slices if necessary.
     *
     * When this callback is enabled, x264_encoder_encode does not return valid NALs;
     * the calling application is expected to acquire all output NALs through the callback.
     *
     * It is generally sensible to combine this callback with a use of slice-max-mbs or
     * slice-max-size.
     *
     * The opaque pointer is the opaque pointer from the input frame associated with this
     * NAL unit. This helps distinguish between nalu_process calls from different sources,
     * e.g. if doing multiple encodes in one process.
     */
    void (*nalu_process)( x264_t *h, x264_nal_t *nal, void *opaque );
} x264_param_t;

typedef struct x264_image_t {
    int     i_csp;       /* Colorspace */
    int     i_plane;     /* Number of image planes */
    int     i_stride[4]; /* Strides for each plane */
    uint8_t *plane[4];   /* Pointers to each plane */
} x264_image_t;

typedef struct x264_image_properties_t {
    /* All arrays of data here are ordered as follows:
     * each array contains one offset per macroblock, in raster scan order.  In interlaced
     * mode, top-field MBs and bottom-field MBs are interleaved at the row level.
     * Macroblocks are 16x16 blocks of pixels (with respect to the luma plane).  For the
     * purposes of calculating the number of macroblocks, width and height are rounded up to
     * the nearest 16.  If in interlaced mode, height is rounded up to the nearest 32 instead. */

    /* In: an array of quantizer offsets to be applied to this image during encoding.
     *     These are added on top of the decisions made by x264.
     *     Offsets can be fractional; they are added before QPs are rounded to integer.
     *     Adaptive quantization must be enabled to use this feature.  Behavior if quant
     *     offsets differ between encoding passes is undefined. */
    float *quant_offsets;
    /* In: optional callback to free quant_offsets when used.
     *     Useful if one wants to use a different quant_offset array for each frame. */
    void (*quant_offsets_free)( void* );

    /* In: optional array of flags for each macroblock.
     *     Allows specifying additional information for the encoder such as which macroblocks
     *     remain unchanged.  Usable flags are listed below.
     *     x264_param_t.analyse.b_mb_info must be set to use this, since x264 needs to track
     *     extra data internally to make full use of this information.
     *
     * Out: if b_mb_info_update is set, x264 will update this array as a result of encoding.
     *
     *      For "MBINFO_CONSTANT", it will remove this flag on any macroblock whose decoded
     *      pixels have changed.  This can be useful for e.g. noting which areas of the
     *      frame need to actually be blitted. Note: this intentionally ignores the effects
     *      of deblocking for the current frame, which should be fine unless one needs exact
     *      pixel-perfect accuracy.
     *
     *      Results for MBINFO_CONSTANT are currently only set for P-frames, and are not
     *      guaranteed to enumerate all blocks which haven't changed.  (There may be false
     *      negatives, but no false positives.)
     */
    uint8_t *mb_info;
    /* In: optional callback to free mb_info when used. */
    void (*mb_info_free)( void* );

    /* The macroblock is constant and remains unchanged from the previous frame. */
    #define X264_MBINFO_CONSTANT   (1U<<0)
    /* More flags may be added in the future. */

    /* Out: SSIM of the the frame luma (if x264_param_t.b_ssim is set) */
    double f_ssim;
    /* Out: Average PSNR of the frame (if x264_param_t.b_psnr is set) */
    double f_psnr_avg;
    /* Out: PSNR of Y, U, and V (if x264_param_t.b_psnr is set) */
    double f_psnr[3];

    /* Out: Average effective CRF of the encoded frame */
    double f_crf_avg;
} x264_image_properties_t;

typedef struct x264_picture_t {
    /* In: force picture type (if not auto)
     *     If x264 encoding parameters are violated in the forcing of picture types,
     *     x264 will correct the input picture type and log a warning.
     * Out: type of the picture encoded */
    int     i_type;
    /* In: force quantizer for != X264_QP_AUTO */
    int     i_qpplus1;
    /* In: pic_struct, for pulldown/doubling/etc...used only if b_pic_struct=1.
     *     use pic_struct_e for pic_struct inputs
     * Out: pic_struct element associated with frame */
    int     i_pic_struct;
    /* Out: whether this frame is a keyframe.  Important when using modes that result in
     * SEI recovery points being used instead of IDR frames. */
    int     b_keyframe;
    /* In: user pts, Out: pts of encoded picture (user)*/
    int64_t i_pts;
    /* Out: frame dts. When the pts of the first frame is close to zero,
     *      initial frames may have a negative dts which must be dealt with by any muxer */
    int64_t i_dts;
    /* In: custom encoding parameters to be set from this frame forwards
           (in coded order, not display order). If NULL, continue using
           parameters from the previous frame.  Some parameters, such as
           aspect ratio, can only be changed per-GOP due to the limitations
           of H.264 itself; in this case, the caller must force an IDR frame
           if it needs the changed parameter to apply immediately. */
    x264_param_t *param;
    /* In: raw image data */
    /* Out: reconstructed image data.  x264 may skip part of the reconstruction process,
            e.g. deblocking, in frames where it isn't necessary.  To force complete
            reconstruction, at a small speed cost, set b_full_recon. */
    x264_image_t img;
    /* In: optional information to modify encoder decisions for this frame
     * Out: information about the encoded frame */
    x264_image_properties_t prop;
    /* Out: HRD timing information. Output only when i_nal_hrd is set. */
    x264_hrd_t hrd_timing;
    /* In: arbitrary user SEI (e.g subtitles, AFDs) */
    x264_sei_t extra_sei;
    /* private user data. copied from input to output frames. */
    void *opaque;
} x264_picture_t;

#define MAX_NUM_NAL 16

typedef struct {
    int*        p_nnal;
    x264_nal_t* p_nal; // max slice count <= (MAX_NUM_NAL-3)
    uint8_t*    p_payload;
} bmx264_nalptr_t;

typedef struct {
    char func[64];
    int  par_num;
    long ret_val;
} bmx264_msg_header_t;

typedef struct x264_msg_par_s {
    int  type; /* 0: 64-bit pointer;
                * 1: 32-bit signed integer;
                * 2: 32-bit unsigned integer;
                * 3: string */
    int  size; // the number of bytes
    char name[16];
} bmx264_msg_par_t;

#define BMX264_MSG_T(N) \
typedef struct { \
        bmx264_msg_header_t header; \
        bmx264_msg_par_t    par[N]; \
} bmx264_msg##N##_t;

BMX264_MSG_T(1);
BMX264_MSG_T(2);
BMX264_MSG_T(3);
BMX264_MSG_T(4);


typedef struct BMX264Context {
    AVClass*        class;

    int             device_idx;
    bm_handle_t     device_handle;
    int32_t         proc_handle;

    bm_device_mem_t msg_d[4];
    uint8_t*        msg_d_va[4];
    int             msg_size[4];

    bmx264_msg4_t   msgN;

    /* host, device 各一份 */
    x264_param_t    params;
    bm_device_mem_t params_d;
    uint8_t*        params_d_va;

    x264_picture_t  pic_s;
    bm_device_mem_t pic_d;
    uint8_t*        pic_d_va;

    x264_picture_t  pic_out_s;
    bm_device_mem_t pic_out_d;
    uint8_t*        pic_out_d_va;

    bm_device_mem_t plane_d;
    uint8_t*        plane_d_va[3];

    bmx264_nalptr_t np_s;
    uint8_t*        nal_s;
    bm_device_mem_t nal_d;
    uint8_t*        nal_d_va;

    /* x264 handle */
    long            enc;

    uint8_t*        sei;
    int             sei_size;

    /* input parameters */
    char*           preset;
    char*           tune;

    int             aud;
    int             mbtree;

    int             forced_idr;

    char*           bmx264_params;

    int             perf;  /* indicate if do the performance testing */
    double          total_time; /* ms */
    long            total_frame;
} BMX264Context;


int bmx264_init(AVCodecContext* avctx, int device_idx, int width, int height);
int bmx264_deinit(AVCodecContext* avctx);

int bmx264_pic_alloc(AVCodecContext* avctx, int y_size, int cbcr_size, int pix_fmt);
int bmx264_pic_free(AVCodecContext* avctx);

long bmx264_dispatch_task(AVCodecContext* avctx, const char func_name[], uint8_t* func_param_va);
long bmx264_dispatch_task2(AVCodecContext* avctx, bmx264_msg_header_t* header, bmx264_msg_par_t par[]);

int bmx264_param_parse(x264_param_t *p, const char *name, const char *value);
char* bmx264_param2string(x264_param_t *p, int b_res);
#endif

#endif /* __BMX264_COMMON_H__ */

