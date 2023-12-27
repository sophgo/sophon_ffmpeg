/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: stitch_uapi.h
 * Description:
 */

#ifndef _U_STITCH_UAPI_H_
#define _U_STITCH_UAPI_H_

#include <linux/cvi_comm_stitch.h>

#ifdef __cplusplus
extern "C" {
#endif

struct stitch_src_frm_cfg {
	__u8 src_id;
	VIDEO_FRAME_INFO_S VideoFrame;
	CVI_S32 MilliSec;
};

struct stitch_chn_frm_cfg {
	VIDEO_FRAME_INFO_S VideoFrame;
	CVI_S32 MilliSec;
};

struct stitch_vb_pool_cfg {
	void *reserved;
	__u32 VbPool;
};

/* Public */

#define CVI_STITCH_INIT          _IO('S',   0x00)
#define CVI_STITCH_DEINIT        _IO('S',   0x01)
#define CVI_STITCH_SET_SRC_ATTR  _IOW('S',  0x02, STITCH_SRC_ATTR_S)
#define CVI_STITCH_GET_SRC_ATTR  _IOWR('S', 0x03, STITCH_SRC_ATTR_S)
#define CVI_STITCH_SET_CHN_ATTR  _IOW('S',  0x04, STITCH_CHN_ATTR_S)
#define CVI_STITCH_GET_CHN_ATTR  _IOWR('S', 0x05, STITCH_CHN_ATTR_S)
#define CVI_STITCH_SET_OP_ATTR   _IOW('S',  0x06, STITCH_OP_ATTR_S)
#define CVI_STITCH_GET_OP_ATTR   _IOWR('S', 0x07, STITCH_OP_ATTR_S)
#define CVI_STITCH_SET_WGT_ATTR   _IOW('S',  0x08, STITCH_WGT_ATTR_S)
#define CVI_STITCH_GET_WGT_ATTR   _IOWR('S', 0x09, STITCH_WGT_ATTR_S)

#define CVI_STITCH_SET_REGX      _IOW('S',  0x0a, CVI_U8)
#define CVI_STITCH_DEV_ENABLE    _IO('S',   0x0b)
#define CVI_STITCH_DEV_DISABLE   _IO('S',   0x0c)
#define CVI_STITCH_SEND_SRC_FRM  _IOW('S',  0x0d, struct stitch_src_frm_cfg)
#define CVI_STITCH_SEND_CHN_FRM  _IOW('S',  0x0e, struct stitch_chn_frm_cfg)
#define CVI_STITCH_GET_CHN_FRM   _IOWR('S', 0x0f, struct stitch_chn_frm_cfg)
#define CVI_STITCH_RLS_CHN_FRM   _IOW('S',  0x10, struct stitch_chn_frm_cfg)
#define CVI_STITCH_ATTACH_VB_POOL _IOW('S', 0x11, struct stitch_vb_pool_cfg)
#define CVI_STITCH_DETACH_VB_POOL _IOW('S', 0x12, struct stitch_vb_pool_cfg)
#define CVI_STITCH_DUMP_REGS _IO('S', 0x13)
#define CVI_STITCH_RST _IO('S', 0x14)

/* Internal use */

#ifdef __cplusplus
}
#endif

#endif /* _U_VPSS_UAPI_H_ */
