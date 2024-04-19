/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: dwa_uapi.h
 * Description:
 */

#ifndef _DWA_UAPI_H_
#define _DWA_UAPI_H_

#include <linux/cvi_comm_gdc.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cvi_dwa_buffer {
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

struct dwa_handle_data {
	__u64 handle;
};

/*
 * stImgIn: Input picture
 * stImgOut: Output picture
 * au64privateData[4]: RW; Private data of task
 * reserved: RW; Debug information,state of current picture
 */
struct dwa_task_attr {
	__u64 handle;

	struct _VIDEO_FRAME_INFO_S stImgIn;
	struct _VIDEO_FRAME_INFO_S stImgOut;
	__u64 au64privateData[4];
	__u32 enRotation;
	__u64 reserved;
	union {
		FISHEYE_ATTR_S stFishEyeAttr;
		AFFINE_ATTR_S stAffineAttr;
		LDC_ATTR_S stLdcAttr;
		WARP_ATTR_S stWarpAttr;
	};

	CVI_U64 meshHandle;
};

struct dwa_identity_attr {
	__u64 handle;
	GDC_IDENTITY_ATTR_S attr;
};

struct dwa_chn_frm_cfg {
	VIDEO_FRAME_INFO_S VideoFrame;
	CVI_S32 MilliSec;
	struct dwa_identity_attr identity;
};

#define CVI_DWA_BEGIN_JOB _IOWR('D', 0x00, struct dwa_handle_data)
#define CVI_DWA_END_JOB _IOW('D', 0x01, struct dwa_handle_data)
#define CVI_DWA_CANCEL_JOB _IOW('D', 0x02, struct dwa_handle_data)
#define CVI_DWA_ADD_ROT_TASK _IOW('D', 0x03, struct dwa_task_attr)
#define CVI_DWA_ADD_LDC_TASK _IOW('D', 0x04, struct dwa_task_attr)
#define CVI_DWA_ADD_COR_TASK _IOW('D', 0x05, struct dwa_task_attr)
#define CVI_DWA_ADD_AFF_TASK _IOW('D', 0x06, struct dwa_task_attr)
#define CVI_DWA_INIT _IO('D', 0x07)
#define CVI_DWA_DEINIT _IO('D', 0x08)
#define CVI_DWA_SET_JOB_IDENTITY _IOW('D', 0x09, struct dwa_identity_attr)
#define CVI_DWA_GET_WORK_JOB _IOR('D', 0x0a, struct dwa_handle_data)
#define CVI_DWA_GET_CHN_FRM _IOWR('D', 0x0b, struct dwa_chn_frm_cfg)
#define CVI_DWA_ADD_WAR_TASK _IOW('D', 0x0c, struct dwa_task_attr)

#ifdef __cplusplus
}
#endif

#endif /* _DWA_UAPI_H_ */
