#ifndef BMCV_API_EXT_H
#define BMCV_API_EXT_H
#define BMCV_VERSION_MAJOR 2
#include "bmlib_runtime.h"
#ifdef _WIN32
#ifndef NULL
  #ifdef __cplusplus
    #define NULL 0
  #else
    #define NULL ((void *)0)
  #endif
#endif
#define DECL_EXPORT __declspec(dllexport)
#define DECL_IMPORT __declspec(dllimport)
#else
#define DECL_EXPORT
#define DECL_IMPORT
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * bmcv api with the new interface.
 */

#ifndef WIN32
typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
} __attribute__((packed)) face_rect_t;
#else
#pragma pack(push, 1)
typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
} face_rect_t;
#pragma pack(pop)
#endif

#define MIN_PROPOSAL_NUM (1)
#define MAX_PROPOSAL_NUM (40000) //(65535)

#ifndef WIN32
typedef struct nms_proposal {
    int          size;
    face_rect_t  face_rect[MAX_PROPOSAL_NUM];
    int          capacity;
    face_rect_t *begin;
    face_rect_t *end;
} __attribute__((packed)) nms_proposal_t;
#else
#pragma pack(push, 1)
typedef struct nms_proposal {
    int          size;
    face_rect_t  face_rect[MAX_PROPOSAL_NUM];
    int          capacity;
    face_rect_t *begin;
    face_rect_t *end;
} nms_proposal_t;
#pragma pack(pop)
#endif

#define MAX_RECT_NUM (8 * 1024)
#ifndef WIN32
typedef struct {
    face_rect_t  face_rect[MAX_RECT_NUM];
    int          size;
    int          capacity;
    face_rect_t *begin;
    face_rect_t *end;
} __attribute__((packed)) m_proposal_t;
#else
#pragma pack(push, 1)
typedef struct {
    face_rect_t  face_rect[MAX_RECT_NUM];
    int          size;
    int          capacity;
    face_rect_t *begin;
    face_rect_t *end;
} m_proposal_t;
#pragma pack(pop)
#endif

// common struct

// BMCV_IMAGE_FOR_IN and BMCV_IMAGE_FOR_OUT may be deprecated in future version.
// We recommend not use this.
#define BMCV_IMAGE_FOR_IN BMCV_HEAP1_ID
#define BMCV_IMAGE_FOR_OUT BMCV_HEAP0_ID

#define BM_STITCH_MAX_SRC_NUM 4

typedef enum bmcv_heap_id_ {
    BMCV_HEAP0_ID = 0,
    BMCV_HEAP1_ID = 1,
    BMCV_HEAP_ANY
} bmcv_heap_id;

typedef enum bm_image_data_format_ext_ {
    DATA_TYPE_EXT_FLOAT32,
    DATA_TYPE_EXT_1N_BYTE,
    DATA_TYPE_EXT_1N_BYTE_SIGNED,
    DATA_TYPE_EXT_FP16,
    DATA_TYPE_EXT_BF16,
    DATA_TYPE_EXT_U16,
    DATA_TYPE_EXT_S16,
    DATA_TYPE_EXT_U32,
} bm_image_data_format_ext;

typedef enum bm_image_format_ext_ {
    FORMAT_YUV420P,
    FORMAT_YUV422P,
    FORMAT_YUV444P,
    FORMAT_NV12,
    FORMAT_NV21,
    FORMAT_NV16,
    FORMAT_NV61,
    FORMAT_NV24,
    FORMAT_RGB_PLANAR,
    FORMAT_BGR_PLANAR,
    FORMAT_RGB_PACKED,
    FORMAT_BGR_PACKED,
    FORMAT_RGBP_SEPARATE,
    FORMAT_BGRP_SEPARATE,
    FORMAT_GRAY,
    FORMAT_COMPRESSED,
    FORMAT_HSV_PLANAR,
    FORMAT_ARGB_PACKED,
    FORMAT_ABGR_PACKED,
    FORMAT_YUV444_PACKED,
    FORMAT_YVU444_PACKED,
    FORMAT_YUV422_YUYV,
    FORMAT_YUV422_YVYU,
    FORMAT_YUV422_UYVY,
    FORMAT_YUV422_VYUY,
    FORMAT_RGBYP_PLANAR,
    FORMAT_HSV180_PACKED,
    FORMAT_HSV256_PACKED,
    FORMAT_BAYER,
} bm_image_format_ext;

typedef enum csc_type {
    CSC_YCbCr2RGB_BT601 = 0,
    CSC_YPbPr2RGB_BT601,
    CSC_RGB2YCbCr_BT601,
    CSC_YCbCr2RGB_BT709,
    CSC_RGB2YCbCr_BT709,
    CSC_RGB2YPbPr_BT601,
    CSC_YPbPr2RGB_BT709,
    CSC_RGB2YPbPr_BT709,
    CSC_USER_DEFINED_MATRIX = 1000,
    CSC_MAX_ENUM
} csc_type_t;

struct bm_image_private;

typedef struct bm_image{
    int                       width;
    int                       height;
    bm_image_format_ext       image_format;
    bm_image_data_format_ext  data_type;
    struct bm_image_private  *image_private = NULL;
} bm_image;

typedef struct bm_image_tensor{
    bm_image image;
    int      image_c;
    int      image_n;
} bm_image_tensor;

typedef struct bm_image_format_info {
    int                      plane_nb;
    bm_device_mem_t          plane_data[8];
    int                      stride[8];
    int                      width;
    int                      height;
    bm_image_format_ext      image_format;
    bm_image_data_format_ext data_type;
    bool                     default_stride;
} bm_image_format_info_t;


// vpp/tpu struct

typedef struct bmcv_convert_to_attr_s {
    float alpha_0;
    float beta_0;
    float alpha_1;
    float beta_1;
    float alpha_2;
    float beta_2;
} bmcv_convert_to_attr;

typedef struct bmcv_rect {
    unsigned int start_x;
    unsigned int start_y;
    unsigned int crop_w;
    unsigned int crop_h;
} bmcv_rect_t;

typedef enum bmcv_resize_algorithm_ {
    BMCV_INTER_NEAREST = 0,
    BMCV_INTER_LINEAR  = 1,
    BMCV_INTER_BICUBIC = 2
} bmcv_resize_algorithm;

typedef struct bmcv_padding_atrr_s {
    unsigned int  dst_crop_stx;
    unsigned int  dst_crop_sty;
    unsigned int  dst_crop_w;
    unsigned int  dst_crop_h;
    unsigned char padding_r;
    unsigned char padding_g;
    unsigned char padding_b;
    int           if_memset;
} bmcv_padding_attr_t;

typedef struct {
    unsigned short csc_coe00;
    unsigned short csc_coe01;
    unsigned short csc_coe02;
    unsigned char csc_add0;
    unsigned short csc_coe10;
    unsigned short csc_coe11;
    unsigned short csc_coe12;
    unsigned char csc_add1;
    unsigned short csc_coe20;
    unsigned short csc_coe21;
    unsigned short csc_coe22;
    unsigned char csc_add2;
} csc_matrix_t;

typedef struct bmcv_copy_to_atrr_s {
    int           start_x;
    int           start_y;
    unsigned char padding_r;
    unsigned char padding_g;
    unsigned char padding_b;
    int           if_padding;
} bmcv_copy_to_atrr_t;

typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} bmcv_color_t;

typedef struct bmcv_resize_s {
    int start_x;
    int start_y;
    int in_width;
    int in_height;
    int out_width;
    int out_height;
} bmcv_resize_t;

typedef struct bmcv_resize_image_s {
    bmcv_resize_t *resize_img_attr;
    int            roi_num;
    unsigned char  stretch_fit;
    unsigned char  padding_b;
    unsigned char  padding_g;
    unsigned char  padding_r;
    unsigned int   interpolation;
} bmcv_resize_image;

typedef struct {
    int x;
    int y;
} bmcv_point_t;

typedef struct bmcv_affine_matrix_s {
    float m[6];
} bmcv_affine_matrix;

typedef struct bmcv_affine_image_matrix_s {
    bmcv_affine_matrix *matrix;
    int                 matrix_num;
} bmcv_affine_image_matrix;

typedef struct bmcv_perspective_matrix_s {
    float m[9];
} bmcv_perspective_matrix;

typedef struct bmcv_perspective_image_matrix_s {
    bmcv_perspective_matrix *matrix;
    int                      matrix_num;
} bmcv_perspective_image_matrix;

typedef struct bmcv_perspective_coordinate_s {
    int x[4];
    int y[4];
} bmcv_perspective_coordinate;

typedef struct bmcv_perspective_image_coordinate_s {
    bmcv_perspective_coordinate *coordinate;
    int                         coordinate_num;
} bmcv_perspective_image_coordinate;

typedef bmcv_affine_image_matrix bmcv_warp_image_matrix;
typedef bmcv_affine_matrix bmcv_warp_matrix;

typedef enum {
    BM_THRESH_BINARY = 0,
    BM_THRESH_BINARY_INV,
    BM_THRESH_TRUNC,
    BM_THRESH_TOZERO,
    BM_THRESH_TOZERO_INV,
    BM_THRESH_TYPE_MAX
} bm_thresh_type_t;

typedef enum bm_cv_nms_alg_ {
    HARD_NMS = 0,
    SOFT_NMS,
    ADAPTIVE_NMS,
    SSD_NMS,
    MAX_NMS_TYPE
} bm_cv_nms_alg_e;


// dpu struct

typedef enum bmcv_dpu_sgbm_mode_{
    DPU_SGBM_MUX0 = 1,             //only sgbm,u8 disp out(no post process),16 align
    DPU_SGBM_MUX1 = 2,             //only sgbm,u16 disp out(post process),16 align
    DPU_SGBM_MUX2 = 3,             //only sgbm,u8 disp out(post process),16 align
} bmcv_dpu_sgbm_mode;

typedef enum bmcv_dpu_online_mode_{
    DPU_ONLINE_MUX0 = 4,  //sgbm 2 fgs online, fgs u8 disp out,16 align
    DPU_ONLINE_MUX1 = 5,  //sgbm 2 fgs online, fgs u16 depth out,32 align
    DPU_ONLINE_MUX2 = 6,  //sgbm 2 fgs online, sgbm u16 depth out,32 align
} bmcv_dpu_online_mode;

typedef enum bmcv_dpu_fgs_mode_{
    DPU_FGS_MUX0 = 7,              //only fgs, u8 disp out,16 align
    DPU_FGS_MUX1 = 8,              //only fgs, u16 depth out,32 align
} bmcv_dpu_fgs_mode;

typedef enum bmcv_dpu_disp_range_{
    BMCV_DPU_DISP_RANGE_DEFAULT,
    BMCV_DPU_DISP_RANGE_16,
    BMCV_DPU_DISP_RANGE_32,
    BMCV_DPU_DISP_RANGE_48,
    BMCV_DPU_DISP_RANGE_64,
    BMCV_DPU_DISP_RANGE_80,
    BMCV_DPU_DISP_RANGE_96,
    BMCV_DPU_DISP_RANGE_112,
    BMCV_DPU_DISP_RANGE_128,
    BMCV_DPU_DISP_RANGE_BUTT
} bmcv_dpu_disp_range;

typedef enum bmcv_dpu_bfw_mode_{
    DPU_BFW_MODE_DEFAULT,
    DPU_BFW_MODE_1x1,
    DPU_BFW_MODE_3x3,
    DPU_BFW_MODE_5x5,
    DPU_BFW_MODE_7x7,
    DPU_BFW_MODE_BUTT
} bmcv_dpu_bfw_mode;

typedef enum bmcv_dpu_depth_unit_{
    BMCV_DPU_DEPTH_UNIT_DEFAULT,
    BMCV_DPU_DEPTH_UNIT_MM,
    BMCV_DPU_DEPTH_UNIT_CM,
    BMCV_DPU_DEPTH_UNIT_DM,
    BMCV_DPU_DEPTH_UNIT_M,
    BMCV_DPU_DEPTH_UNIT_BUTT
} bmcv_dpu_depth_unit;

typedef enum bmcv_dpu_dcc_dir_{
    BMCV_DPU_DCC_DIR_DEFAULT,
    BMCV_DPU_DCC_DIR_A12,
    BMCV_DPU_DCC_DIR_A13,
    BMCV_DPU_DCC_DIR_A14,
    BMCV_DPU_DCC_DIR_BUTT
} bmcv_dpu_dcc_dir;

typedef struct bmcv_dpu_sgbm_attrs_{
    bmcv_dpu_bfw_mode    bfw_mode_en;
    bmcv_dpu_disp_range  disp_range_en;
    unsigned short       disp_start_pos;
    unsigned int         dpu_census_shift;
    unsigned int         dpu_rshift1;
    unsigned int         dpu_rshift2;
    bmcv_dpu_dcc_dir     dcc_dir_en;
    unsigned int         dpu_ca_p1;
    unsigned int         dpu_ca_p2;
    unsigned int         dpu_uniq_ratio;
    unsigned int         dpu_disp_shift;
} bmcv_dpu_sgbm_attrs;

typedef struct bmcv_dpu_fgs_attrs_{
    unsigned int         fgs_max_count;
    unsigned int         fgs_max_t;
    unsigned int         fxbase_line;
    bmcv_dpu_depth_unit  depth_unit_en;
} bmcv_dpu_fgs_attrs;


// ldc/dwa/blend struct

typedef enum bmcv_rot_mode_ {
    BMCV_ROTATION_0 = 0,
    BMCV_ROTATION_90,
    BMCV_ROTATION_180,
    BMCV_ROTATION_270,
    BMCV_ROTATION_XY_FLIP,
    BMCV_ROTATION_MAX
} bmcv_rot_mode;

typedef struct _bmcv_gdc_attr {
    bool bAspect; /* RW;Whether aspect ration  is keep */
    int s32XRatio; /* RW; Range: [0, 100], field angle ration of  horizontal,valid when bAspect=0.*/
    int s32YRatio; /* RW; Range: [0, 100], field angle ration of  vertical,valid when bAspect=0.*/
    int s32XYRatio; /* RW; Range: [0, 100], field angle ration of  all,valid when bAspect=1.*/
    int s32CenterXOffset;
    int s32CenterYOffset;
    int s32DistortionRatio;
} bmcv_gdc_attr;

enum bm_stitch_wgt_mode {
    BM_STITCH_WGT_YUV_SHARE = 0,
    BM_STITCH_WGT_UV_SHARE,
    BM_STITCH_WGT_SEP,
};

struct bm_stitch_src_ovlp_attr {
    short ovlp_lx[BM_STITCH_MAX_SRC_NUM -1];
    short ovlp_rx[BM_STITCH_MAX_SRC_NUM -1];
};

struct bm_stitch_src_bd_attr {
    short bd_lx[BM_STITCH_MAX_SRC_NUM];
    short bd_rx[BM_STITCH_MAX_SRC_NUM];
};

struct stitch_param {
    struct bm_stitch_src_ovlp_attr ovlap_attr;
    struct bm_stitch_src_bd_attr bd_attr;
    bm_device_mem_t wgt_phy_mem[BM_STITCH_MAX_SRC_NUM -1][2];
    enum bm_stitch_wgt_mode wgt_mode;
};


// ive struct

#define BM_IVE_MAX_REGION_NUM       254

typedef struct bmcv_ive_add_attr_s {
    unsigned short param_x;
    unsigned short param_y;
} bmcv_ive_add_attr;

typedef enum frame2op_mod {
    MOD_BYP = 0,
    MOD_ADD,
    MOD_AND,
    MOD_SUB,
    MOD_OR,
    MOD_XOR,
} bm_ive_frame2op_mod;

typedef enum bm_ive_sub_mode_e{
    IVE_SUB_ABS = 0x0,   // Absolute value of the difference
    IVE_SUB_SHIFT = 0x1, // The output result is obtained by s_fting the result one digit right to reserve the signed bit
    IVE_SUB_BUTT
} bm_ive_sub_mode;

typedef struct bmcv_ive_sub_attr_s {
    bm_ive_sub_mode enMode;
} bmcv_ive_sub_attr;

/*
  IVE_THRESH_BINARY:       srcVal <= lowThr, dstVal = minVal; srcVal > lowThr, dstVal = maxVal.
  IVE_THRESH_TRUNC:        srcVal <= lowThr, dstVal = srcVal; srcVal > lowThr, dstVal = maxVal.
  IVE_THRESH_TO_MINAL:     srcVal <= lowThr, dstVal = minVal; srcVal > lowThr, dstVal = srcVal.
  IVE_THRESH_MIN_MID_MAX: srcVal <= lowThr, dstVal = minVal;  lowThr < srcVal <= _ghThr,
                          dstVal = midVal; srcVal > _ghThr, dstVal = maxVal.
  IVE_THRESH_ORI_MID_MAX: srcVal <= lowThr, dstVal = srcVal;  lowThr < srcVal <= _ghThr,
                          dstVal = midVal; srcVal > _ghThr, dstVal = maxVal.
  IVE_THRESH_MIN_MID_ORI: srcVal <= lowThr, dstVal = minVal;  lowThr < srcVal <= _ghThr,
                          dstVal = midVal; srcVal > _ghThr, dstVal = srcVal.
  IVE_THRESH_MIN_ORI_MAX: srcVal <= lowThr, dstVal = minVal;  lowThr < srcVal <= _ghThr,
                          dstVal = srcVal; srcVal > _ghThr, dstVal = maxVal.
  IVE_THRESH_ORI_MID_ORI: srcVal <= lowThr, dstVal = srcVal;  lowThr < srcVal <= _ghThr,
                          dstVal = midVal; srcVal > _ghThr, dstVal = srcVal.
*/

typedef enum bm_ive_thresh_mode_e{
    // u8
    IVE_THRESH_BINARY = 0x0,
    IVE_THRESH_TRUNC = 0x1,
    IVE_THRESH_TO_MINAL = 0x2,
    IVE_THRESH_MIN_MID_MAX = 0x3,
    IVE_THRESH_ORI_MID_MAX = 0x4,
    IVE_THRESH_MIN_MID_ORI = 0x5,
    IVE_THRESH_MIN_ORI_MAX = 0x6,
    IVE_THRESH_ORI_MID_ORI = 0x7,
    IVE_THRESH_BUTT = 0x8,

    // s16
    IVE_THRESH_S16_TO_S8_MIN_MID_MAX = 0x9,
    IVE_THRESH_S16_TO_S8_MIN_ORI_MAX = 0x10,
    IVE_THRESH_S16_TO_U8_MIN_MID_MAX = 0x11,
    IVE_THRESH_S16_TO_U8_MIN_ORI_MAX = 0x12,
    IVE_THRESH_S16_BUTT = 0x13,

    // u16
    IVE_THRESH_U16_TO_U8_MIN_MID_MAX = 0x14,
    IVE_THRESH_U16_TO_U8_MIN_ORI_MAX = 0x15,
    IVE_THRESH_U16_BUTT

} bm_ive_thresh_mode;

typedef enum bm_ive_thresh_op_mode_e {
    MOD_U8 = 0,
    MOD_U16,
    MOD_S16,
} bm_ive_thresh_op_mode;

typedef struct bmcv_ive_thresh_attr_s {
    int low_thr;
    int high_thr;
    int min_val;
    int mid_val;
    int max_val;
} bmcv_ive_thresh_attr;

typedef enum bm_ive_map_mode_s{
    IVE_MAP_U8 = 0x0,
    IVE_MAP_S16 = 0x1,
    IVE_MAP_U16 = 0x2,
    IVE_MAP_BUTT
} bm_ive_map_mode;

typedef enum bm_ive_dma_mode_e {
    IVE_DMA_DIRECT_COPY = 0x0,
    IVE_DMA_INTERVAL_COPY = 0x1,
    IVE_DMA_SET_3BYTE = 0x2,
    IVE_DMA_SET_8BYTE = 0x3
} bm_ive_dma_mode;

typedef struct bmcv_ive_dma_attr_s {
    unsigned long long val;
    unsigned char hor_seg_size;
    unsigned char elem_size;
    unsigned char ver_seg_rows;
} bmcv_ive_dma_attr;

typedef enum _ive_integ_out_ctrl_e {
    IVE_INTEG_MODE_COMBINE = 0x0,
    IVE_INTEG_MODE_SUM = 0x1,
    IVE_INTEG_MODE_SQSUM = 0x2,
    IVE_INTEG_MODE_BUTT
} ive_integ_out_ctrl_e;

typedef struct bmcv_integ_ctrl_t{
    ive_integ_out_ctrl_e enOutCtrl;
} bmcv_integ_ctrl_s;

typedef struct _bmcv_ive_ncc_dst_mem_s{
    unsigned long long u64Numerator;
    unsigned long long u64QuadSum1;
    unsigned long long u64QuadSum2;
    unsigned char u8Reserved[8];
}bmcv_ive_ncc_dst_mem_t;

typedef enum _bmcv_ord_stat_filter_mode_e{
    BM_IVE_ORD_STAT_FILTER_MEDIAN = 0x0,
    BM_IVE_ORD_STAT_FILTER_MAX    = 0x1,
    BM_IVE_ORD_STAT_FILTER_MIN    = 0x2,
} bmcv_ord_stat_filter_mode;

typedef enum bm_lbp_cmp_mode_e{
    BM_IVE_LBP_CMP_MODE_NORMAL = 0x0, /* P(x)-P(center)>= un8BitThr.s8Val, s(x)=1; else s(x)=0; */
    BM_IVE_LBP_CMP_MODE_ABS    = 0x1, /* Abs(P(x)-P(center))>=un8BitThr.u8Val, s(x)=1; else s(x)=0; */
} bm_lbp_cmp_mode;

typedef union bm_ive_8bit_u{
    signed char  s8Val;
    unsigned char u8Val;
} bm_ive_8bit;

typedef struct bm_lbp_ctrl_attr_s{
    bm_lbp_cmp_mode enMode;
    bm_ive_8bit     un8BitThr;
} bm_lbp_ctrl_attr;

typedef struct bm_ive_dilate_attr_s{
    unsigned char au8Mask[25];  /*The template parameter value must be 0 or 255.*/
} bm_ive_dilate_attr;

typedef bm_ive_dilate_attr bm_ive_erode_attr;

typedef enum bm_ive_magAndAng_outCtrl_e{
    BM_IVE_MAG_AND_ANG_OUT_MAG = 0x0,  /*Only the magnitude is output.*/
    BM_IVE_MAG_AND_ANG_OUT_ALL = 0X1,  /*The magnitude and angle are output.*/
} bm_ive_magAndAng_outCtrl;

typedef struct bm_ive_magAndAng_ctrl_s{
    bm_ive_magAndAng_outCtrl   enOutCtrl;
    unsigned short             u16Thr;
    signed char                as8Mask[25];  /*Template paramter*/
} bm_ive_magAndAng_ctrl;

typedef enum bm_ive_sobel_out_mode_e{
    BM_IVE_SOBEL_OUT_MODE_BOTH = 0x0, /*Output horizontal and vertical*/
    BM_IVE_SOBEL_OUT_MODE_HOR  = 0x1, /*Output horizontal*/
    BM_IVE_SOBEL_OUT_MODE_VER  = 0X2, /*Output vertical*/
} bm_ive_sobel_out_mode;

typedef struct bm_ive_sobel_ctrl_s{
    bm_ive_sobel_out_mode sobelMode;  // output mode
    signed char as8Mask[25];          // Template parameter
} bm_ive_sobel_ctrl;

typedef enum bm_ive_normgrad_outmode_e{
    BM_IVE_NORM_GRAD_OUT_HOR_AND_VER = 0x0,
    BM_IVE_NORM_GRAD_OUT_HOR         = 0x1,
    BM_IVE_NORM_GRAD_OUT_VER         = 0x2,
    BM_IVE_NORM_GRAD_OUT_COMBINE     = 0x3,
} bm_ive_normgrad_outmode;

typedef struct bm_ive_normgrad_ctrl_s{
    bm_ive_normgrad_outmode enMode;
    signed char as8Mask[25];
    unsigned char u8Norm;
} bm_ive_normgrad_ctrl;

typedef struct bm_ive_gmm_ctrl_s{
    unsigned int u22q10NoiseVar;    // Initial noise Variance
    unsigned int u22q10MaxVar;      // Max  Variance
    unsigned int u22q10MinVar;      // Min  Variance
    unsigned short u0q16LearnRate;  // Learning rate
    unsigned short u0q16BgRatio;    // Background ratio
    unsigned short u8q8VarThr;      // Variance Threshold
    unsigned short u0q16InitWeight; // Initial Weight
    unsigned char u8ModelNum;       // Model number: 3 or 5
} bm_ive_gmm_ctrl;

typedef enum gmm2_sns_factor_mode_e{
    SNS_FACTOR_MODE_GLB = 0x0, // Global sensitivity factor mode
    SNS_FACTOR_MODE_PIX = 0x1, // Pixel sensitivity factor mode
} gmm2_sns_factor_mode;

typedef enum gmm2_life_update_factor_mode_e{
    LIFE_UPDATE_FACTOR_MODE_GLB = 0x0, // Global life update factor mode
    LIFE_UPDATE_FACTOR_MODE_PIX = 0x1, // Pixel life update factor mode
} gmm2_life_update_factor_mode;

typedef struct bm_ive_gmm2_ctrl_s{
    gmm2_sns_factor_mode enSnsFactorMode; // Sensitivity factor mode
    gmm2_life_update_factor_mode enLifeUpdateFactorMode; // Life update factor mode
    unsigned short u16GlbLifeUpdateFactor;         // Global life update factor (default: 4)
    unsigned short u16LifeThr;                     // Life threshold (default: 5000)
    unsigned short u16FreqInitVal;                 // Initial frequency (default: 20000)
    unsigned short u16FreqReduFactor;              // Frequency reduction factor (default: 0xFF00)
    unsigned short u16FreqAddFactor;               // Frequency adding factor (default: 0xEF)
    unsigned short u16FreqThr;                     // Frequency threshold (default: 12000)
    unsigned short u16VarRate;                     // Variation update rate (default: 1)
    unsigned short u9q7MaxVar;                     // Max variation (default: (16 * 16)<<7)
    unsigned short u9q7MinVar;                     // Min variation (default: ( 8 *  8)<<7)
    unsigned char u8GlbSnsFactor;                  // Global sensitivity factor (default: 8)
    unsigned char u8ModelNum;                      // Model number (range: 1~5, default: 3)
} bm_ive_gmm2_ctrl;

typedef struct bm_ive_point_u16_s{
    unsigned short u16X;
    unsigned short u16Y;
} bm_ive_point_u16;

typedef struct bm_ive_canny_stack_size_s{
    unsigned int u32StackSize;
    unsigned char u8Reserved[12];
} bm_ive_canny_stack_size;

typedef struct bm_ive_canny_hys_edge_ctrl_s{
    bm_device_mem_t  stMem;
    unsigned short u16LowThr;
    unsigned short u16HighThr;
    signed char as8Mask[25];
} bm_ive_canny_hys_edge_ctrl;

typedef struct bm_ive_filter_ctrl_s{
    signed char as8Mask[25];  // Template parameter filter coefficient
    unsigned char u8Norm;     // Normalization parameter, by right s_ft
} bm_ive_filter_ctrl;

typedef enum bm_ive_csc_mode_e{
    /*CSC: YUV2RGB, video transfer mode, RGB value range [16, 235]*/
    BM_IVE_VIDEO_BT601_YUV2RGB = 0x0,

    /*CSC: YUV2RGB, video transfer mode, RGB value range [16, 235]*/
    BM_IVE_VIDEO_BT709_YUV2RGB = 0x1,

    /*CSC: YUV2RGB, picture transfer mode, RGB value range [0, 255]*/
    BM_IVE_PIC_BT601_YUV2RGB = 0x2,

    /*CSC: YUV2RGB, picture transfer mode, RGB value range [0, 255]*/
    BM_IVE_PIC_BT709_YUV2RGB = 0x3,

    /*CSC: YUV2HSV, picture transfer mode, HSV value range [0, 255]*/
    BM_IVE_PIC_BT601_YUV2HSV = 0x4,

    /*CSC: YUV2HSV, picture transfer mode, HSV value range [0, 255]*/
    BM_IVE_PIC_BT709_YUV2HSV = 0x5,

    /*CSC: YUV2LAB, picture transfer mode, Lab value range [0, 255]*/
    BM_IVE_PIC_BT601_YUV2LAB = 0x6,

    /*CSC: YUV2LAB, picture transfer mode, Lab value range [0, 255]*/
    BM_IVE_PIC_BT709_YUV2LAB = 0x7,

    /*CSC: RGB2YUV, video transfer mode, YUV value range [0, 255]*/
    BM_IVE_VIDEO_BT601_RGB2YUV = 0x8,

    /*CSC: RGB2YUV, video transfer mode, YUV value range [0, 255]*/
    BM_IVE_VIDEO_BT709_RGB2YUV = 0x9,

    /*CSC: RGB2YUV, picture transfer mode, Y:[16, 235],U\V:[16, 240]*/
    BM_IVE_PIC_BT601_RGB2YUV = 0xa,

    /*CSC: RGB2YUV, picture transfer mode, Y:[16, 235],U\V:[16, 240]*/
    BM_IVE_PIC_BT709_RGB2YUV = 0xb,
    BM_IVE_PIC_RGB2HSV = 0xb,
    BM_IVE_PIC_RGB2GRAY = 0xc,
} bm_ive_csc_mode;

typedef enum bm_ive_resize_mode_e{
    IVE_RESIZE_LINEAR = 0,  // Bilinear interpolation
    IVE_RESIZE_AREA = 1,    // Area-based (or super) interpolation
} bm_ive_resize_mode;

typedef struct bm_ive_resize_attr_s{
    bm_ive_resize_mode   enMode;
    unsigned short       u16Num;
} bm_ive_resize_attr;

typedef struct bm_ive_stCandiCorner_attr_s{
    bm_device_mem_t stMem;
    unsigned char   u0q8QualityLevel;
} bm_ive_stCandiCorner_attr;

typedef struct bm_ive_st_max_eig_s{
    unsigned short u16MaxEig;       // S_-Tomasi second step output MaxEig
    unsigned char  u8Reserved[14];  // For 16 byte align
} bm_ive_st_max_eig;

typedef enum bm_ive_gradFg_mode_e{
    GRAD_FG_MODE_USE_CUR_GRAD = 0x0,
    GRAD_FG_MODE_FIND_MIN_GRAD = 0x1,
} bm_ive_gradFg_mode;

typedef struct bm_ive_gradFg_attr_s{
    bm_ive_gradFg_mode enMode;    // Calculation mode
    unsigned short u16EdwFactor;  // Edge width adjustment factor (range: 500 to 2000; default: 1000)
    unsigned char u8CrlCoefThr;   // Gradient vector correlation coefficient threshold (ranges: 50 to 100; default: 80)
    unsigned char u8MagCrlThr;    // Gradient amplitude threshold (range: 0 to 20; default: 4)
    unsigned char u8MinMagDiff;   // Gradient magnitude difference threshold (range: 2 to 8; default: 2)
    unsigned char u8NoiseVal;     // Gradient amplitude noise threshold (range: 1 to 8; default: 1)
    unsigned char u8EdwDark;      // Black pixels enable flag (range: 0 (no), 1 (yes); default: 1)
} bm_ive_gradFg_attr;

typedef enum bm_ive_sad_mode_e{
    BM_IVE_SAD_MODE_MB_4X4 = 0x0,   // 4x4
    BM_IVE_SAD_MODE_MB_8X8 = 0x1,   // 8x8
    BM_IVE_SAD_MODE_MB_16X16 = 0x2, // 16x16
} bm_ive_sad_mode;

typedef enum bm_ive_sad_out_ctrl_s{
    BM_IVE_SAD_OUT_16BIT_BOTH = 0x0,   // Output 16 bit sad and thresh
    BM_IVE_SAD_OUT_8BIT_BOTH  = 0x1,   // Output 8 bit sad and thresh
    BM_IVE_SAD_OUT_16BIT_SAD  = 0x2,   // Output 16 bit sad
    BM_IVE_SAD_OUT_8BIT_SAD   = 0x3,   // Output 8 bit sad
    BM_IVE_SAD_OUT_THRESH     = 0x4,   // Output thresh,16 bits sad
} bm_ive_sad_out_ctrl;

typedef struct bm_ive_sad_attr_s{
    bm_ive_sad_mode enMode;
    bm_ive_sad_out_ctrl enOutCtrl;
    unsigned short u16Thr;   // srcVal <= u16Thr, dstVal = minVal; srcVal > u16Thr, dstVal = maxVal
    unsigned char  u8MinVal; // max value
    unsigned char  u8MaxVal; // min value
} bm_ive_sad_attr;

typedef struct bm_ive_work_bg_pix_s{
    unsigned short u8q4f4Mean; // 0# background grays value
    unsigned short u16AccTime; // ackground cumulative access time
    unsigned char u8PreGray;   // Gray value of last pixel
    unsigned char u5q3DiffThr; // Differential threshold
    unsigned char u8AccFlag;   // Background access flag
    unsigned char u8BgGray[3]; // 1# ~ 3# background grays value
} bm_ive_work_bg_pix;

typedef struct bm_ive_candi_bg_pix_s{
    unsigned short u8q4f4Mean;       // Candidate background grays value
    unsigned short u16StartTime;     // Candidate Background start time
    unsigned short u16SumAccessTime; // Candidate Background cumulative access time
    unsigned short u16ShortKeepTime; // Candidate background short hold time
    unsigned char  u8ChgCond;        // Time condition for candidate background into the changing state
    unsigned char  u8PotenBgLife;    // Potential background cumulative access time
} bm_ive_candi_bg_pix;

typedef struct bm_ive_bg_life_s{
    unsigned char u8WorkBgLife[3]; // 1# ~ 3# background vitality
    unsigned char u8CandiBgLife;   // Candidate background vitality
} bm_ive_bg_life;

typedef struct bm_ive_bg_model_pix_s{
    bm_ive_work_bg_pix stWorkBgPixel; // Working background
    bm_ive_candi_bg_pix stCandiPixel; // Candidate background
    bm_ive_bg_life stBgLife;          // Background vitality
} bm_ive_bg_model_pix;

typedef struct bm_ive_bg_stat_data_s{
    unsigned int u32PixNum;
    unsigned int u32SumLum;
    unsigned char u8Reserved[8];
} bm_ive_bg_stat_data;

typedef struct bm_ive_match_bgmodel_attr_s{
    unsigned int u32CurFrmNum;  // Current frame timestamp, in frame units
    unsigned int u32PreFrmNum;  // Previous frame timestamp, in frame units
    unsigned short u16TimeThr;  // Potential background replacement time threshold (range: 2 to 100 frames; default: 20)

    /*
     * Correlation coefficients between differential threshold and gray value
     * (range: 0 to 5; default: 0)
     */
    unsigned char u8DiffThrCrlCoef;
    unsigned char u8DiffMaxThr; // Maximum of background differential threshold (range: 3 to 15; default: 6)
    unsigned char u8DiffMinThr; // Minimum of background differential threshold (range: 3 to 15; default: 4)
    unsigned char u8DiffThrInc; // Dynamic Background differential threshold increment (range: 0 to 6; default: 0)
    unsigned char u8FastLearnRate; // Quick background learning rate (range: 0 to 4; default: 2)
    unsigned char u8DetChgRegion;   // Whether to detect change region (range: 0 (no), 1 (yes); default: 0)
} bm_ive_match_bgmodel_attr;

typedef struct bm_ive_update_bgmodel_attr_s{
    unsigned int u32CurFrmNum;    // Current frame timestamp, in frame units
    unsigned int u32PreChkTime;   // The last time when background status is checked
    unsigned int u32FrmChkPeriod; // Background status checking period (range: 0 to 2000 frames; default: 50)
    unsigned int u32InitMinTime;  // Background initialization shortest time (range: 20 to 6000 frames; default: 100)
    /*
     * Steady background integration shortest time
     * (range: 20 to 6000 frames; default: 200)
     */
    unsigned int u32StyBgMinBlendTime;
    /*
     * Steady background integration longest time
     * (range: 20 to 40000 frames; default: 1500)
     */
    unsigned int u32StyBgMaxBlendTime;
    /*
     * Dynamic background integration shortest time
     * (range: 0 to 6000 frames; default: 0)
     */

    unsigned int u32DynBgMinBlendTime;
    unsigned int u32StaticDetMinTime; // Still detection shortest time (range: 20 to 6000 frames; default: 80)
    unsigned short u16FgMaxFadeTime;  // Foreground disappearing longest time (range: 1 to 255 seconds; default: 15)
    unsigned short u16BgMaxFadeTime;  // Background disappearing longest time (range: 1 to 255  seconds ; default: 60)

    unsigned char u8StyBgAccTimeRateThr; // Steady background access time ratio threshold (range: 10 to 100; default: 80)
    unsigned char u8ChgBgAccTimeRateThr; // Change background access time ratio threshold (range: 10 to 100; default: 60)
    unsigned char u8DynBgAccTimeThr;     // Dynamic background access time ratio threshold (range: 0 to 50; default: 0)
    unsigned char u8DynBgDepth;          // Dynamic background depth (range: 0 to 3; default: 3)

    /*
     * Background state time ratio threshold when initializing
     * (range: 90 to 100; default: 90)
     */
    unsigned char u8BgEffStaRateThr;
    unsigned char u8AcceBgLearn;  // Whether to accelerate background learning (range: 0 (no), 1 (yes); default: 0)
    unsigned char u8DetChgRegion; // Whether to detect change region (range: 0 (no), 1 (yes); default: 0)
} bm_ive_update_bgmodel_attr;

typedef enum bm_ive_ccl_mode_e{
    BM_IVE_CCL_MODE_4C = 0x0, // 4-connected
    BM_IVE_CCL_MODE_8C = 0x1, // 8-connected
} bm_ive_ccl_mode;

typedef struct bm_ive_ccl_attr_s{
    bm_ive_ccl_mode enMode;          // ccl mode
    unsigned short  u16InitAreaThr;  // Init threshold of region area
    unsigned short  u16Step;         // Increase area step for once
} bm_ive_ccl_attr;

typedef struct bm_ive_region_s{
    int u32Area;               // Represented by the pixel number
    unsigned short u16Left;    // Circumscribed rectangle left border
    unsigned short u16Right;   // Circumscribed rectangle right border
    unsigned short u16Top;     // Circumscribed rectangle top border
    unsigned short u16Bottom;  // Circumscribed rectangle bottom border
} bm_ive_region;

typedef struct bm_ive_ccblob_s{
    unsigned short  u16CurAeraThr;  // Threshold of the result regions' area
    signed char     s8LabelStatus;  // -1: Labeled failed ; 0: Labeled successfully
    unsigned char   u8RegionNum;    // Number of valid region, non-continuous stored
    bm_ive_region   astRegion[BM_IVE_MAX_REGION_NUM]; // Valid regions with 'u32Area>0' and 'label = ArrayIndex+1
} bm_ive_ccblob;



// common api

/** bm_image_create
 * @brief Create and fill bm_image structure
 * @param [in] handle                     The bm handle which return by
 * bm_dev_request.
 * @param [in] img_h                      The height or rows of the creating
 * image.
 * @param [in] img_w                     The width or cols of the creating
 * image.
 * @param [in] image_format      The image_format of the creating image,
 *  please choose one from bm_image_format_ext enum.
 * @param [in] data_type               The data_type of the creating image,
 *  be caution that not all combinations between image_format and data_type
 *  are supported.
 * @param [in] stride                        the stride array for planes, each
 * number in array means corresponding plane pitch stride in bytes. The plane
 * size is determinated by image_format. If this array is null, we may use
 * default value.
 *  @param [out] image                   The filled bm_image structure.
 *  For example, we need create a 480x480 NV12 format image, we know that NV12
 * format has 2 planes, we need pitch stride is 256 aligned(just for example) so
 * the pitch stride for the first plane is 512, so as the same for the second
 * plane.
 * The call may as following
 * bm_image res;
 *  int stride[] = {512, 512};
 *  bm_image_create(handle, 480, 480, FORMAT_NV12, DATA_TYPE_EXT_1N_BYTE, &res,
 * stride); If bm_image_create return BM_SUCCESS, res is created successfully.
 */
DECL_EXPORT bm_status_t bm_image_create(
    bm_handle_t              handle,
    int                      img_h,
    int                      img_w,
    bm_image_format_ext      image_format,
    bm_image_data_format_ext data_type,
    bm_image *               image,
    int *                    stride = NULL);

/** bm_image_destroy
 * @brief Destroy bm_image and free the corresponding system memory and device
 * memory.
 * @param [in] image                     The bm_image structure ready to
 * destroy. If bm_image_destroy return BM_SUCCESS, image is destroy successfully
 * and the corresponding system memory and device memory are freed.
 */
DECL_EXPORT bm_status_t bm_image_destroy(bm_image *image);
DECL_EXPORT bm_handle_t bm_image_get_handle(bm_image *image);
DECL_EXPORT bm_status_t bm_image_copy_host_to_device(bm_image image, void *buffers[]);
DECL_EXPORT bm_status_t bm_image_copy_device_to_host(bm_image image, void *buffers[]);
DECL_EXPORT bm_status_t bm_image_attach(bm_image image, bm_device_mem_t *device_memory);
DECL_EXPORT bm_status_t bm_image_detach(bm_image image);
DECL_EXPORT bool        bm_image_is_attached(bm_image);
DECL_EXPORT int         bm_image_get_plane_num(bm_image);
DECL_EXPORT bm_status_t bm_image_get_stride(bm_image image, int *stride);
DECL_EXPORT bm_status_t bm_image_get_format_info(bm_image *image, bm_image_format_info_t *info);
DECL_EXPORT bm_status_t bm_image_alloc_dev_mem(bm_image image, int heap_id = BMCV_HEAP_ANY);
DECL_EXPORT bm_status_t bm_image_alloc_dev_mem_heap_mask(bm_image image, int heap_mask);
DECL_EXPORT bm_status_t bm_image_get_byte_size(bm_image image, int *size);
DECL_EXPORT bm_status_t bm_image_get_device_mem(bm_image image, bm_device_mem_t *mem);
DECL_EXPORT bm_status_t bm_image_alloc_contiguous_mem(int image_num, bm_image *images, int heap_id = BMCV_HEAP_ANY);
DECL_EXPORT bm_status_t bm_image_alloc_contiguous_mem_heap_mask(int image_num, bm_image *images, int heap_mask);
DECL_EXPORT bm_status_t bm_image_free_contiguous_mem(int image_num, bm_image *images);
DECL_EXPORT bm_status_t bm_image_attach_contiguous_mem(int image_num, bm_image *images, bm_device_mem_t dmem);
DECL_EXPORT bm_status_t bm_image_detach_contiguous_mem(int image_num, bm_image *images);
DECL_EXPORT bm_status_t bm_image_get_contiguous_device_mem(int image_num, bm_image *images, bm_device_mem_t *mem);
DECL_EXPORT bm_status_t bm_image_write_to_bmp(bm_image image, const char *filename);
DECL_EXPORT void bm_read_bin(bm_image src, const char *input_name);
DECL_EXPORT void bm_write_bin(bm_image dst, const char *output_name);
DECL_EXPORT bm_status_t bmcv_width_align(bm_handle_t handle, bm_image input, bm_image output);

bm_status_t bm_image_tensor_get_device_mem(bm_image_tensor  image_tensor,
                                           bm_device_mem_t *mem);
bool bm_image_tensor_is_attached(bm_image_tensor image_tensor);

bm_status_t bm_image_tensor_destroy(bm_image_tensor image_tensor);

bm_status_t bm_image_tensor_alloc_dev_mem(bm_image_tensor image_tensor,
                                          int             heap_id);

bm_status_t concat_images_to_tensor(bm_handle_t      handle,
                                    int              image_num,
                                    bm_image *       images,
                                    bm_image_tensor *image_tensor);

DECL_EXPORT void algorithm_to_str(bmcv_resize_algorithm algorithm, char* res);

DECL_EXPORT void format_to_str(bm_image_format_ext format, char* res);

DECL_EXPORT int is_csc_yuv_or_rgb(bm_image_format_ext format);


// vpp/tpu/jpu api

DECL_EXPORT bm_status_t bmcv_image_convert_to(
    bm_handle_t          handle,
    int                  input_num,
    bmcv_convert_to_attr convert_to_attr,
    bm_image *           input,
    bm_image *           output);


DECL_EXPORT bm_status_t bmcv_image_vpp_convert(
    bm_handle_t           handle,
    int                   output_num,
    bm_image              input,
    bm_image *            output,
    bmcv_rect_t *         crop_rect = NULL,
    bmcv_resize_algorithm algorithm = BMCV_INTER_LINEAR);

DECL_EXPORT bm_status_t bmcv_image_vpp_basic(
    bm_handle_t           handle,
    int                   in_img_num,
    bm_image*             input,
    bm_image*             output,
    int*                  crop_num_vec = NULL,
    bmcv_rect_t*          crop_rect = NULL,
    bmcv_padding_attr_t*  padding_attr = NULL,
    bmcv_resize_algorithm algorithm = BMCV_INTER_LINEAR,
    csc_type_t            csc_type = CSC_MAX_ENUM,
    csc_matrix_t*         matrix = NULL);

DECL_EXPORT bm_status_t bmcv_image_vpp_csc_matrix_convert(
    bm_handle_t           handle,
    int                   output_num,
    bm_image              input,
    bm_image*             output,
    csc_type_t            csc,
    csc_matrix_t*         matrix = NULL,
    bmcv_resize_algorithm algorithm = BMCV_INTER_LINEAR,
    bmcv_rect_t*          crop_rect = NULL);

DECL_EXPORT bm_status_t bmcv_image_storage_convert(
    bm_handle_t      handle,
    int              image_num,
    bm_image*        input_,
    bm_image*        output_);

DECL_EXPORT bm_status_t bmcv_image_storage_convert_with_csctype(
    bm_handle_t      handle,
    int              image_num,
    bm_image*        input_,
    bm_image*        output_,
    csc_type_t       csc_type);

DECL_EXPORT bm_status_t bmcv_image_vpp_convert_padding(
    bm_handle_t             handle,
    int                     output_num,
    bm_image                input,
    bm_image*               output,
    bmcv_padding_attr_t*    padding_attr,
    bmcv_rect_t*            crop_rect = NULL,
    bmcv_resize_algorithm   algorithm = BMCV_INTER_LINEAR);

DECL_EXPORT bm_status_t bmcv_image_resize(
    bm_handle_t          handle,
    int                  input_num,
    bmcv_resize_image    resize_attr[],
    bm_image *           input,
    bm_image *           output);

DECL_EXPORT bm_status_t bmcv_image_draw_rectangle(
    bm_handle_t   handle,
    bm_image      image,
    int           rect_num,
    bmcv_rect_t * rects,
    int           line_width,
    unsigned char r,
    unsigned char g,
    unsigned char b);

DECL_EXPORT bm_status_t bmcv_image_csc_convert_to(
    bm_handle_t             handle,
    int                     img_num,
    bm_image*               input,
    bm_image*               output,
    int*                    crop_num_vec,
    bmcv_rect_t*            crop_rect,
    bmcv_padding_attr_t*    padding_attr,
    bmcv_resize_algorithm   algorithm,
    csc_type_t              csc_type,
    csc_matrix_t*           matrix,
    bmcv_convert_to_attr*   convert_to_attr);

DECL_EXPORT bm_status_t bmcv_image_copy_to(
    bm_handle_t         handle,
    bmcv_copy_to_atrr_t copy_to_attr,
    bm_image            input,
    bm_image            output);

DECL_EXPORT bm_status_t bmcv_image_vpp_stitch(
    bm_handle_t           handle,
    int                   input_num,
    bm_image*             input,
    bm_image              output,
    bmcv_rect_t*          dst_crop_rect,
    bmcv_rect_t*          src_crop_rect = NULL,
    bmcv_resize_algorithm algorithm = BMCV_INTER_LINEAR);

DECL_EXPORT bm_status_t bmcv_image_mosaic(
    bm_handle_t           handle,
    int                   mosaic_num,
    bm_image              input,
    bmcv_rect_t *         mosaic_rect,
    int                   is_expand);

DECL_EXPORT bm_status_t bmcv_image_fill_rectangle(
    bm_handle_t             handle,
    bm_image                image,
    int                     rect_num,
    bmcv_rect_t *           rects,
    unsigned char           r,
    unsigned char           g,
    unsigned char           b);

DECL_EXPORT bm_status_t bmcv_image_watermark_superpose(
    bm_handle_t           handle,
    bm_image *            image,
    bm_device_mem_t *     bitmap_mem,
    int                   bitmap_num,
    int                   bitmap_type,
    int                   pitch,
    bmcv_rect_t *         rects,
    bmcv_color_t          color);

DECL_EXPORT bm_status_t bmcv_image_watermark_repeat_superpose(
    bm_handle_t         handle,
    bm_image            image,
    bm_device_mem_t     bitmap_mem,
    int                 bitmap_num,
    int                 bitmap_type,
    int                 pitch,
    bmcv_rect_t *       rects,
    bmcv_color_t        color);

// quality_factor = 84
DECL_EXPORT bm_status_t bmcv_image_jpeg_enc(
    bm_handle_t handle,
    int         image_num,
    bm_image *  src,
    void **     p_jpeg_data,
    size_t *    out_size,
    int         quality_factor = 85);

DECL_EXPORT bm_status_t bmcv_image_jpeg_dec(
    bm_handle_t handle,
    void **     p_jpeg_data,
    size_t *    in_size,
    int         image_num,
    bm_image *  dst);

DECL_EXPORT bm_status_t bmcv_image_absdiff(
    bm_handle_t handle,
    bm_image input1,
    bm_image input2,
    bm_image output);

DECL_EXPORT bm_status_t  bmcv_image_axpy(
    bm_handle_t handle,
    bm_device_mem_t tensor_A,
    bm_device_mem_t tensor_X,
    bm_device_mem_t tensor_Y,
    bm_device_mem_t tensor_F,
    int input_n,
    int input_c,
    int input_h,
    int input_w);

DECL_EXPORT bm_status_t bmcv_image_add_weighted(
    bm_handle_t handle,
    bm_image input1,
    float alpha,
    bm_image input2,
    float beta,
    float gamma,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_bitwise_and(
    bm_handle_t handle,
    bm_image input1,
    bm_image input2,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_bitwise_or(
    bm_handle_t handle,
    bm_image input1,
    bm_image input2,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_bitwise_xor(
    bm_handle_t handle,
    bm_image input1,
    bm_image input2,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_sobel(
    bm_handle_t handle,
    bm_image input,
    bm_image output,
    int dx,
    int dy,
    int ksize = 3,
    float scale = 1,
    float delta = 0);

DECL_EXPORT bm_status_t bmcv_image_gaussian_blur(
    bm_handle_t handle,
    bm_image input,
    bm_image output,
    int kw,
    int kh,
    float sigmaX,
    float sigmaY = 0);

DECL_EXPORT bm_status_t bmcv_hamming_distance(
    bm_handle_t handle,
    bm_device_mem_t input1,
    bm_device_mem_t input2,
    bm_device_mem_t output,
    int bits_len,
    int input1_num,
    int input2_num);

DECL_EXPORT bm_status_t bmcv_sort(
    bm_handle_t     handle,
    bm_device_mem_t src_index_addr,
    bm_device_mem_t src_data_addr,
    int             data_cnt,
    bm_device_mem_t dst_index_addr,
    bm_device_mem_t dst_data_addr,
    int             sort_cnt,
    int             order,
    bool            index_enable,
    bool            auto_index);

DECL_EXPORT bm_status_t bmcv_image_threshold(
    bm_handle_t handle,
    bm_image input,
    bm_image output,
    unsigned char thresh,
    unsigned char max_value,
    bm_thresh_type_t type);

DECL_EXPORT bm_status_t bmcv_min_max(
    bm_handle_t handle,
    bm_device_mem_t input,
    float *minVal,
    float *maxVal,
    int len);

DECL_EXPORT bm_status_t bmcv_cmulp(
    bm_handle_t handle,
    bm_device_mem_t inputReal,
    bm_device_mem_t inputImag,
    bm_device_mem_t pointReal,
    bm_device_mem_t pointImag,
    bm_device_mem_t outputReal,
    bm_device_mem_t outputImag,
    int batch,
    int len);

DECL_EXPORT bm_status_t bmcv_image_warp_affine(
    bm_handle_t              handle,
    int                      image_num,
    bmcv_affine_image_matrix matrix[4],
    bm_image *               input,
    bm_image *               output,
    int                      use_bilinear = 0);

DECL_EXPORT bm_status_t bmcv_image_warp_affine_similar_to_opencv(
    bm_handle_t              handle,
    int                      image_num,
    bmcv_affine_image_matrix matrix[4],
    bm_image *               input,
    bm_image *               output,
    int                      use_bilinear);

DECL_EXPORT bm_status_t bmcv_image_warp_perspective(
    bm_handle_t                   handle,
    int                           image_num,
    bmcv_perspective_image_matrix matrix[4],
    bm_image *                    input,
    bm_image *                    output,
    int                           use_bilinear = 0);

DECL_EXPORT bm_status_t bmcv_image_warp_perspective_with_coordinate(
    bm_handle_t                       handle,
    int                               image_num,
    bmcv_perspective_image_coordinate coordinate[4],
    bm_image *                        input,
    bm_image *                        output,
    int                               use_bilinear = 0);

DECL_EXPORT bm_status_t bmcv_image_warp_perspective_similar_to_opencv(
    bm_handle_t                       handle,
    int                               image_num,
    bmcv_perspective_image_matrix     matrix[4],
    bm_image *                        input,
    bm_image *                        output,
    int                               use_bilinear);

DECL_EXPORT bm_status_t bmcv_base64_enc(
    bm_handle_t     handle,
    bm_device_mem_t src,
    bm_device_mem_t dst,
    unsigned long   len[2]);

DECL_EXPORT bm_status_t bmcv_base64_dec(
    bm_handle_t     handle,
    bm_device_mem_t src,
    bm_device_mem_t dst,
    unsigned long   len[2]);

DECL_EXPORT bm_status_t bmcv_image_put_text(
    bm_handle_t handle,
    bm_image image,
    const char* text,
    bmcv_point_t org,
    bmcv_color_t color,
    float fontScale,
    int thickness);

DECL_EXPORT bm_status_t bmcv_nms(
    bm_handle_t     handle,
    bm_device_mem_t input_proposal_addr,
    int             proposal_size,
    float           nms_threshold,
    bm_device_mem_t output_proposal_addr);

DECL_EXPORT bm_status_t bmcv_nms_ext(
    bm_handle_t     handle,
    bm_device_mem_t input_proposal_addr,
    int             proposal_size,
    float           nms_threshold,
    bm_device_mem_t output_proposal_addr,
    int             topk            = 1,
    float           score_threshold = 0.0f,
    int             nms_alg         = HARD_NMS,
    float           sigma = 1.0,
    int             weighting_method = 0,
    float         * densities = NULL,
    float           eta = 0.0f);

DECL_EXPORT bm_status_t bmcv_image_draw_lines(
    bm_handle_t handle,
    bm_image img,
    const bmcv_point_t* start,
    const bmcv_point_t* end,
    int line_num,
    bmcv_color_t color,
    int thickness);

DECL_EXPORT bm_status_t bmcv_fft_1d_create_plan(
    bm_handle_t handle,
    int batch,
    int len,
    bool forward,
    void *plan);

DECL_EXPORT bm_status_t bmcv_fft_2d_create_plan(
    bm_handle_t handle,
    int M,
    int N,
    bool forward,
    void *plan);

DECL_EXPORT bm_status_t bmcv_fft_execute(
    bm_handle_t handle,
    bm_device_mem_t inputReal,
    bm_device_mem_t inputImag,
    bm_device_mem_t outputReal,
    bm_device_mem_t outputImag,
    const void *plan);

DECL_EXPORT bm_status_t bmcv_fft_execute_real_input(
    bm_handle_t handle,
    bm_device_mem_t inputReal,
    bm_device_mem_t outputReal,
    bm_device_mem_t outputImag,
    const void *plan);

DECL_EXPORT void bmcv_fft_destroy_plan(bm_handle_t handle, void *plan);

DECL_EXPORT bm_status_t bmcv_distance(
    bm_handle_t handle,
    bm_device_mem_t input,
    bm_device_mem_t output,
    int dim,
    const void * pnt,
    int len,
    int dtyte);

DECL_EXPORT bm_status_t bmcv_image_laplacian(
    bm_handle_t handle,
    bm_image input,
    bm_image output,
    unsigned int ksize);

DECL_EXPORT bm_status_t bmcv_as_strided(
    bm_handle_t handle,
    bm_device_mem_t input,
    bm_device_mem_t output,
    int input_row,
    int input_col,
    int output_row,
    int output_col,
    int row_stride,
    int col_stride);

DECL_EXPORT bm_status_t bmcv_image_transpose(
    bm_handle_t handle,
    bm_image input,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_pyramid_down(
    bm_handle_t handle,
    bm_image input,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_image_bayer2rgb(
    bm_handle_t handle,
    unsigned char* convd_kernel,
    bm_image input,
    bm_image output);

DECL_EXPORT bm_status_t bmcv_batch_topk(
    bm_handle_t     handle,
    bm_device_mem_t src_data_addr,
    bm_device_mem_t src_index_addr,
    bm_device_mem_t dst_data_addr,
    bm_device_mem_t dst_index_addr,
    bm_device_mem_t buffer_addr,
    bool            src_index_valid,
    int             k,
    int             batch,
    int*            per_batch_cnt,
    bool            same_batch_cnt,
    int             src_batch_stride,
    bool            descending);

DECL_EXPORT bm_status_t bmcv_matmul(
    bm_handle_t      handle,
    int              M,
    int              N,
    int              K,
    bm_device_mem_t  A,
    bm_device_mem_t  B,
    bm_device_mem_t  C,
    int              A_sign, /*1: signed 0: unsigned */
    int              B_sign,
    int              rshift_bit,
    int              result_type, /* 0:8bit 1:int16 2:fp32 */
    bool             is_B_trans,
    float            alpha = 1,
    float            beta = 0);

DECL_EXPORT bm_status_t bmcv_calc_hist(
    bm_handle_t handle,
    bm_device_mem_t input,
    bm_device_mem_t output,
    int C,
    int H,
    int W,
    const int *channels,
    int dims,
    const int *histSizes,
    const float *ranges,
    int inputDtype);

DECL_EXPORT bm_status_t bmcv_calc_hist_with_weight(
    bm_handle_t handle,
    bm_device_mem_t input,
    bm_device_mem_t output,
    const float *weight,
    int C,
    int H,
    int W,
    const int *channels,
    int dims,
    const int *histSizes,
    const float *ranges,
    int inputDtype);


// dpu api

DECL_EXPORT bm_status_t bmcv_dpu_sgbm_disp(
    bm_handle_t         handle,
    bm_image            *left_image,
    bm_image            *right_image,
    bm_image            *disp_image,
    bmcv_dpu_sgbm_attrs *dpu_attr,
    bmcv_dpu_sgbm_mode  sgbm_mode);

DECL_EXPORT bm_status_t bmcv_dpu_fgs_disp(
    bm_handle_t         handle,
    bm_image            *guide_image,
    bm_image            *smooth_image,
    bm_image            *disp_image,
    bmcv_dpu_fgs_attrs  *fgs_attr,
    bmcv_dpu_fgs_mode   fgs_mode);

DECL_EXPORT bm_status_t bmcv_dpu_online_disp(
    bm_handle_t            handle,
    bm_image               *left_image,
    bm_image               *right_image,
    bm_image               *disp_image,
    bmcv_dpu_sgbm_attrs    *dpu_attr,
    bmcv_dpu_fgs_attrs     *fgs_attr,
    bmcv_dpu_online_mode   online_mode);


// ldc/dwa/blend api

DECL_EXPORT bm_status_t bmcv_ldc_rot(
    bm_handle_t          handle,
    bm_image             in_image,
    bm_image             out_image,
    bmcv_rot_mode        rot_mode);

DECL_EXPORT bm_status_t bmcv_ldc_gdc(
    bm_handle_t          handle,
    bm_image             in_image,
    bm_image             out_image,
    bmcv_gdc_attr        ldc_attr);

DECL_EXPORT bm_status_t bmcv_ldc_gdc_gen_mesh(
    bm_handle_t          handle,
    bm_image             in_image,
    bm_image             out_image,
    bmcv_gdc_attr        ldc_attr,
    bm_device_mem_t      dmem);

DECL_EXPORT bm_status_t bmcv_ldc_gdc_load_mesh(
    bm_handle_t          handle,
    bm_image             in_image,
    bm_image             out_image,
    bm_device_mem_t      dmem);

DECL_EXPORT bm_status_t bmcv_dwa_rot(
    bm_handle_t          handle,
    bm_image             input_image,
    bm_image             output_image,
    bmcv_rot_mode        rot_mode);

DECL_EXPORT bm_status_t bmcv_dwa_gdc(
    bm_handle_t          handle,
    bm_image             input_image,
    bm_image             output_image,
    bmcv_gdc_attr        ldc_attr);

DECL_EXPORT bm_status_t bmcv_blending(
    bm_handle_t handle,
    int       input_num,
    bm_image* input,
    bm_image  output,
    struct stitch_param stitch_config);


// ive api

DECL_EXPORT bm_status_t bmcv_image_ive_frame2op(
    bm_handle_t          handle,
    void *               attr,
    bm_ive_frame2op_mod  opType,
    bm_image *           input1,
    bm_image *           input2,
    bm_image *           output);

DECL_EXPORT bm_status_t bmcv_image_ive_thresh(
    bm_handle_t             handle,
    bm_ive_thresh_op_mode   opType,
    bm_ive_thresh_mode      threshMode,
    bmcv_ive_thresh_attr *  attr,
    bm_image *              input,
    bm_image *              output);

DECL_EXPORT bm_status_t bmcv_image_ive_dma(
    bm_handle_t             handle,
    bm_ive_dma_mode         dma_mode,
    bmcv_ive_dma_attr *     attr,
    bm_image *              input,
    bm_image *              output);

DECL_EXPORT bm_status_t bmcv_image_ive_map(
    bm_handle_t             handle,
    bm_ive_map_mode         map_mode,
    bm_image *              input,
    bm_image *              output);

DECL_EXPORT bm_status_t bmcv_image_ive_hist(
    bm_handle_t          handle,
    bm_image *           input,
    bm_device_mem_t*     output,
    int                  hist_size);

DECL_EXPORT bm_status_t bmcv_image_ive_integ(
    bm_handle_t          handle,
    bm_image *           input,
    bm_device_mem_t*     output,
    bmcv_integ_ctrl_s*   integ_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_ncc(
    bm_handle_t          handle,
    bm_image *           input1,
    bm_image *           input2,
    bm_device_mem_t*     output);

DECL_EXPORT bm_status_t bmcv_image_ive_ordStatFilter(
    bm_handle_t               handle,
    bm_image *                input,
    bm_image *                output,
    bmcv_ord_stat_filter_mode ordStatFilter_mode);

DECL_EXPORT bm_status_t bmcv_image_ive_lbp(
    bm_handle_t          handle,
    bm_image *           input,
    bm_image *           output,
    bm_lbp_ctrl_attr *   lbp_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_dilate(
    bm_handle_t           handle,
    bm_image *            input,
    bm_image *            output,
    bm_ive_dilate_attr *  dilate_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_erode(
    bm_handle_t           handle,
    bm_image *            input,
    bm_image *            output,
    bm_ive_erode_attr *  erode_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_magAndAng(
    bm_handle_t             handle,
    bm_image *              input,
    bm_image *              magOutput,
    bm_image *              angOutput,
    bm_ive_magAndAng_ctrl * magAndAng_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_sobel(
    bm_handle_t         handle,
    bm_image *          input,
    bm_image *          hOutput,
    bm_image *          vOutput,
    bm_ive_sobel_ctrl * sobel_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_normgrad(
    bm_handle_t            handle,
    bm_image *             input,
    bm_image *             output_h,
    bm_image *             output_v,
    bm_image *             output_hv,
    bm_ive_normgrad_ctrl * normgrad_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_gmm(
    bm_handle_t          handle,
    bm_image *           input,
    bm_image *           output_fg,
    bm_image *           output_bg,
    bm_device_mem_t *    output_model,
    bm_ive_gmm_ctrl *    gmm_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_gmm2(
    bm_handle_t          handle,
    bm_image *           input,
    bm_image *           input_factor,
    bm_image *           output_fg,
    bm_image *           output_bg,
    bm_image *           output_match_model_info,
    bm_device_mem_t *    output_model,
    bm_ive_gmm2_ctrl *   gmm2_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_canny(
    bm_handle_t                  handle,
    bm_image *                   input,
    bm_device_mem_t              output_edge,
    bm_ive_canny_hys_edge_ctrl * canny_hys_edge_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_filter(
    bm_handle_t                  handle,
    bm_image *                   input,
    bm_image *                   output,
    bm_ive_filter_ctrl *         filter_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_csc(
    bm_handle_t     handle,
    bm_image *      input,
    bm_image *      output,
    bm_ive_csc_mode csc_mode);

DECL_EXPORT bm_status_t bmcv_image_ive_resize(
    bm_handle_t            handle,
    bm_ive_resize_attr *   resize_attr,
    bm_image *             input,
    bm_image *             output);

DECL_EXPORT bm_status_t bmcv_image_ive_stCandiCorner(
    bm_handle_t      handle,
    bm_image *       input,
    bm_image *       output,
    bm_ive_stCandiCorner_attr * stCandiCorner_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_gradfg(
    bm_handle_t           handle,
    bm_image *            input_bgDiffFg,
    bm_image *            input_curGrad,
    bm_image *            intput_bgGrad,
    bm_image *            output_gradFg,
    bm_ive_gradFg_attr *  gradfg_attr);

DECL_EXPORT bm_status_t bmcv_imgae_ive_sad(
    bm_handle_t           handle,
    bm_image *            input1,
    bm_image *            input2,
    bm_image *            output_sad,
    bm_image *            output_thr,
    bm_ive_sad_attr *     sad_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_match_bgmodel(
    bm_handle_t          handle,
    bm_image *           src_cur_img,
    bm_image *           src_bgmodel_img,
    bm_image *           src_fgflag_img,
    bm_image *           dst_diff_fg_img,
    bm_device_mem_t *    dst_stat_data_mem,
    bm_ive_match_bgmodel_attr * matchBgmodel_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_update_bgmodel(
    bm_handle_t          handle,
    bm_image *           src_bgmodel_img,
    bm_image *           src_fgflag_img,
    bm_image *           dst_bg_img,
    bm_image *           dst_chgsta_img,
    bm_device_mem_t  *   dst_stat_data_mem,
    bm_ive_update_bgmodel_attr * updateBgmodel_attr);

DECL_EXPORT bm_status_t bmcv_image_ive_ccl(
    bm_handle_t        handle,
    bm_image *         src_dst_image,
    bm_device_mem_t *  ccblob_output,
    bm_ive_ccl_attr *  ccl_attr);

#if defined(__cplusplus)
}
#endif

#endif
