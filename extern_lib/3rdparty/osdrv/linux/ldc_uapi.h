/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: ldc_uapi.h
 * Description:
 */

#ifndef _U_LDC_UAPI_H_
#define _U_LDC_UAPI_H_

#include <linux/cvi_comm_gdc.h>

#ifdef __cplusplus
extern "C" {
#endif


enum cvi_ldc_op {
	CVI_LDC_OP_NONE,
	CVI_LDC_OP_XY_FLIP,
	CVI_LDC_OP_ROT_90,
	CVI_LDC_OP_ROT_270,
	CVI_LDC_OP_LDC,
	CVI_LDC_OP_MAX,
};

struct cvi_ldc_buffer {
	__u8 pixel_fmt; // 0: Y only, 1: NV21
	__u8 rot;
	__u16 bgcolor; // data outside start/end if used in operation

	__u16 src_width; // src width, including padding
	__u16 src_height; // src height, including padding

	__u32 src_y_base;
	__u32 src_c_base;
	__u32 dst_y_base;
	__u32 dst_c_base;

	__u32 map_base;
};

struct cvi_ldc_rot {
	__u64 handle;

	void *pUsageParam;
	void *vb_in;
	__u32 enPixFormat;
	__u64 mesh_addr;
	__u8 sync_io;
	void *cb;
	void *pcbParam;
	__u32 cbParamSize;
	__u32 enModId;
	__u32 enRotation;
};

struct gdc_handle_data {
	__u64 handle;
};

/*
 * stImgIn: Input picture
 * stImgOut: Output picture
 * au64privateData[4]: RW; Private data of task
 * reserved: RW; Debug information,state of current picture
 */
struct gdc_task_attr {
	__u64 handle;

	struct _VIDEO_FRAME_INFO_S stImgIn;
	struct _VIDEO_FRAME_INFO_S stImgOut;
	__u64 au64privateData[4];
	__u32 enRotation;
	__u64 reserved;
	union {
		FISHEYE_ATTR_S stFishEyeAttr;
		AFFINE_ATTR_S stAffineAttr;
		LDC_ATTR_S stLDCAttr;
	};

	CVI_U64 meshHandle;
	struct _LDC_BUF_WRAP_S stBufWrap;
	CVI_U32 bufWrapDepth;
	CVI_U64 bufWrapPhyAddr;
};

struct gdc_identity_attr {
	__u64 handle;
	GDC_IDENTITY_ATTR_S attr;
};

struct gdc_chn_frm_cfg {
	VIDEO_FRAME_INFO_S VideoFrame;
	CVI_S32 MilliSec;
	struct gdc_identity_attr identity;
};

struct ldc_buf_wrap_cfg {
	__u64 handle;
	struct gdc_task_attr stTask;
	struct _LDC_BUF_WRAP_S stBufWrap;
};

struct ldc_vb_pool_cfg {
	void *reserved;
	__u32 VbPool;
};

#define CVI_LDC_BEGIN_JOB _IOWR('L', 0x00, struct gdc_handle_data)
#define CVI_LDC_END_JOB _IOW('L', 0x01, struct gdc_handle_data)
#define CVI_LDC_CANCEL_JOB _IOW('L', 0x02, unsigned long long)
#define CVI_LDC_ADD_ROT_TASK _IOW('L', 0x03, struct gdc_task_attr)
#define CVI_LDC_ADD_LDC_TASK _IOW('L', 0x04, struct gdc_task_attr)
#define CVI_LDC_INIT _IO('L', 0x05)
#define CVI_LDC_DEINIT _IO('L', 0x06)
#define CVI_LDC_SET_JOB_IDENTITY _IOW('L', 0x07, struct gdc_identity_attr)
#define CVI_LDC_GET_WORK_JOB _IOR('L', 0x08, struct gdc_handle_data)
#define CVI_LDC_GET_CHN_FRM _IOWR('L', 0x09, struct gdc_chn_frm_cfg)

#define CVI_LDC_SET_BUF_WRAP _IOW('L', 0x0a, struct ldc_buf_wrap_cfg)
#define CVI_LDC_GET_BUF_WRAP _IOWR('L', 0x0b, struct ldc_buf_wrap_cfg)
#define CVI_LDC_ATTACH_VB_POOL _IOW('L', 0x0c, struct ldc_vb_pool_cfg)
#define CVI_LDC_DETACH_VB_POOL _IO('L', 0x0d)

#ifdef __cplusplus
}
#endif

#endif /* _U_LDC_UAPI_H_ */
