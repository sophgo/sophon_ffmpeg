/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File name: dwa_uapi.h
 * Description:
 */

#ifndef _DWA_UAPI_H_
#define _DWA_UAPI_H_

#include <linux/comm_gdc.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cvi_dwa_buffer {
	unsigned char pixel_fmt; // 0: Y only, 1: NV21
	unsigned char rot;
	unsigned short bgcolor; // data outside start/end if used in operation

	unsigned short src_width; // src width, including padding
	unsigned short src_height; // src height, including padding

	unsigned int src_y_base;
	unsigned int src_c_base;
	unsigned int dst_y_base;
	unsigned int dst_c_base;

	unsigned int map_base;
};

struct dwa_handle_data {
	unsigned long long handle;
};

/*
 * img_in: Input picture
 * img_out: Output picture
 * private_data[4]: RW; Private data of task
 * reserved: RW; Debug information,state of current picture
 */
struct dwa_task_attr {
	unsigned long long handle;

	video_frame_info_s img_in;
	video_frame_info_s img_out;
	unsigned long long private_data[4];
	unsigned int rotation;
	unsigned long long reserved;
	union {
		fisheye_attr_s fisheye_attr;
		affine_attr_s affine_attr;
		ldc_attr_s stLdcAttr;
		warp_attr_s stWarpAttr;
	};

	unsigned long long mesh_handle;
};

struct dwa_identity_attr {
	unsigned long long handle;
	gdc_identity_attr_s attr;
};

struct dwa_chn_frm_cfg {
	video_frame_info_s video_frame;
	int milli_sec;
	struct dwa_identity_attr identity;
};

#define DWA_BEGIN_JOB _IOWR('D', 0x00, struct dwa_handle_data)
#define DWA_END_JOB _IOW('D', 0x01, struct dwa_handle_data)
#define DWA_CANCEL_JOB _IOW('D', 0x02, struct dwa_handle_data)
#define DWA_ADD_ROT_TASK _IOW('D', 0x03, struct dwa_task_attr)
#define DWA_ADD_LDC_TASK _IOW('D', 0x04, struct dwa_task_attr)
#define DWA_ADD_COR_TASK _IOW('D', 0x05, struct dwa_task_attr)
#define DWA_ADD_AFF_TASK _IOW('D', 0x06, struct dwa_task_attr)
#define DWA_INIT _IO('D', 0x07)
#define DWA_DEINIT _IO('D', 0x08)
#define DWA_SET_JOB_IDENTITY _IOW('D', 0x09, struct dwa_identity_attr)
#define DWA_GET_WORK_JOB _IOR('D', 0x0a, struct dwa_handle_data)
#define DWA_GET_CHN_FRM _IOWR('D', 0x0b, struct dwa_chn_frm_cfg)
#define DWA_ADD_WAR_TASK _IOW('D', 0x0c, struct dwa_task_attr)
#define DWA_SUSPEND _IO('D',0x0d)
#define DWA_RESUME _IO('D',0x0e)

#ifdef __cplusplus
}
#endif

#endif /* _DWA_UAPI_H_ */
