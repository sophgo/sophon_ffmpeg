/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: include/cvi_comm_stitch.h
 * Description:
 *   The common data type defination for Stitch module.
 */

#ifndef __CVI_COMM_STITCH_H__
#define __CVI_COMM_STITCH_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include <linux/cvi_type.h>
#include <linux/cvi_common.h>
#include <linux/cvi_comm_video.h>
#include <linux/cvi_defines.h>

#define STITCH_R_IDX 0
#define STITCH_G_IDX 1
#define STITCH_B_IDX 2

#define STITCH_ALIGN 16

enum stitch_src_id {
	STITCH_SRC_ID_0 = 0,
	STITCH_SRC_ID_1,
	STITCH_SRC_ID_2,
	STITCH_SRC_ID_3,
	STITCH_SRC_ID_SEP,
};

enum stitch_wgt_mode {
	STITCH_WGT_YUV_SHARE = 0,
	STITCH_WGT_UV_SHARE,
	STITCH_WGT_SEP,
};

enum stitch_data_src {
	STITCH_DATA_SRC_DDR = 0,
	STITCH_DATA_SRC_VPSS,
	STITCH_DATA_SRC_SEP,
};

enum stitch_way_num {
	STITCH_2_WAY = 0,
	STITCH_4_WAY,
	STITCH_WAY_SEP,
};

//ovlp_lx[0]: overlap left x position of img12
//ovlp_rx[0]: overlap right x position of img12
//ovlp_lx[1]: overlap left x position of img23
//ovlp_rx[1]: overlap right x position of img23
//ovlp_lx[2]: overlap left x position of img34
//ovlp_rx[2]: overlap right x position of img34
struct stitch_src_ovlp_attr {
	short ovlp_lx[STITCH_MAX_SRC_NUM -1];
	short ovlp_rx[STITCH_MAX_SRC_NUM -1];
};

//bd_lx[0]: left boardWidth of img1
//bd_rx[0]: right boardWidth of img1
struct stitch_src_bd_attr {
	short bd_lx[STITCH_MAX_SRC_NUM];
	short bd_rx[STITCH_MAX_SRC_NUM];
};

struct stitch_src_attr {
	struct stitch_src_ovlp_attr ovlap_attr;
	struct stitch_src_bd_attr bd_attr;
	SIZE_S size[STITCH_MAX_SRC_NUM];
	PIXEL_FORMAT_E fmt_in;
	enum stitch_way_num way_num;
};

//phy_addr_wgt[0]: phy_addr_wgt12(alpha, beta)
//phy_addr_wgt[1]: phy_addr_wgt23(alpha, beta)
//phy_addr_wgt[2]: phy_addr_wgt34(alpha, beta)
//size_wgt[0]: size of wgt12(alpha, beta)
//size_wgt[1]: size of wgt23(alpha, beta)
//size_wgt[2]: size of wgt34(alpha, beta)
struct stitch_bld_wgt_attr {
	__u64 phy_addr_wgt[STITCH_MAX_SRC_NUM -1][2];
	SIZE_S size_wgt[STITCH_MAX_SRC_NUM -1];
};

struct stitch_chn_attr {
	SIZE_S size;
	PIXEL_FORMAT_E fmt_out;
};

struct stitch_op_attr {
	enum stitch_wgt_mode wgt_mode;
	enum stitch_data_src data_src;
};

typedef struct stitch_src_attr STITCH_SRC_ATTR_S;
typedef struct stitch_chn_attr STITCH_CHN_ATTR_S;
typedef struct stitch_op_attr STITCH_OP_ATTR_S;
typedef struct stitch_bld_wgt_attr STITCH_WGT_ATTR_S;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
#endif /* __CVI_COMM_STITCH_H__ */
