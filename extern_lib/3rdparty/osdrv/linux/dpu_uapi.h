/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: dpu_uapi.h
 * Description:
 */

#ifndef _U_DPU_UAPI_H_
#define _U_DPU_UAPI_H_

#include <linux/cvi_comm_dpu.h>
#include <linux/cvi_comm_sys.h>
#include <linux/cvi_defines.h>

#ifdef __cplusplus
extern "C" {
#endif


struct dpu_grp_cfg {
	DPU_GRP DpuGrp;
};

struct dpu_grp_attr {
	DPU_GRP DpuGrp;
	DPU_GRP_ATTR_S stGrpAttr;
};

struct dpu_chn_cfg {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
};

struct dpu_chn_attr {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
	DPU_CHN_ATTR_S stChnAttr;
};

struct dpu_set_frame_cfg {
	DPU_GRP DpuGrp;
	VIDEO_FRAME_INFO_S stSrcLeftFrame;
	VIDEO_FRAME_INFO_S stSrcRightFrame;
	__s32 s32MilliSec;
};

struct dpu_get_frame_cfg {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
	VIDEO_FRAME_INFO_S stFrameInfo;
	__s32 s32MilliSec;
};

struct dpu_release_frame_cfg {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
	VIDEO_FRAME_INFO_S stFrameInfo;
};

struct dpu_vb_pool_cfg {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
	__u32 hVbPool;
};

struct dpu_snap_cfg {
	DPU_GRP DpuGrp;
	DPU_CHN DpuChn;
	__u32 frame_cnt;
};


/* Public */
#define CVI_DPU_CREATE_GROUP _IOW('U', 0x00, struct dpu_grp_attr)
#define CVI_DPU_DESTROY_GROUP _IOW('U', 0x01, struct dpu_grp_cfg)
#define CVI_DPU_GET_AVAIL_GROUP _IOWR('U', 0x11, DPU_GRP)
#define CVI_DPU_START_GROUP _IOW('U', 0x02, struct dpu_grp_cfg)
#define CVI_DPU_STOP_GROUP _IOW('u', 0x03, struct dpu_grp_cfg)
#define CVI_DPU_SET_GRP_ATTR _IOW('U', 0x05, struct dpu_grp_attr)
#define CVI_DPU_GET_GRP_ATTR _IOWR('U', 0x06, struct dpu_grp_attr)

#define CVI_DPU_SET_CHN_ATTR _IOW('U', 0x7, struct dpu_chn_attr)
#define CVI_DPU_GET_CHN_ATTR _IOWR('U', 0x8, struct dpu_chn_attr)

#define CVI_DPU_ENABLE_CHN _IOW('U', 0x9, struct dpu_chn_cfg)
#define CVI_DPU_DISABLE_CHN _IOW('U', 0xa, struct dpu_chn_cfg)

#define CVI_DPU_GET_FRAME _IOWR('U', 0x0b, struct dpu_get_frame_cfg)
#define CVI_DPU_RELEASE_FRAME _IOW('U', 0x0c, struct dpu_release_frame_cfg)
#define CVI_DPU_SEND_FRAME _IOW('U', 0x0d, struct dpu_set_frame_cfg)

#define CVI_DPU_CHECK_REG_READ _IO('U', 0x0e)
#define CVI_DPU_CHECK_REG_WRITE _IO('U', 0x0f)
#define CVI_DPU_SEND_CHN_FRAME _IOW('U', 0x10, struct dpu_get_frame_cfg)

#define CVI_DPU_GET_SGBM_STATUS _IO('U', 0xa0)
#define CVI_DPU_GET_FGS_STATUS _IO('U', 0xa1)


#ifdef __cplusplus
}
#endif

#endif /* _U_DPU_UAPI_H_ */
