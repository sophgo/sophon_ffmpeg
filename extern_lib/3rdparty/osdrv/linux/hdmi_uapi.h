/*
 * Copyright (C) Cvitek Co., Ltd. 2019-2020. All rights reserved.
 *
 * File Name: hdmi_uapi.h
 * Description:
 */

#ifndef _U_HDMI_UAPI_H_
#define _U_HDMI_UAPI_H_


#include <linux/cvi_comm_sys.h>
#include <linux/cvi_common.h>
#include <linux/cvi_comm_hdmi.h>


/* Public */
#define CVI_HDMI_INIT _IO('H', 0x00)
#define CVI_HDMI_DEINIT _IO('H', 0x01)
#define CVI_HDMI_OPEN _IO('H', 0x02)
#define CVI_HDMI_CLOSE _IO('H', 0x03)
#define CVI_HDMI_GET_SINK_CAPABILITY _IOWR('H', 0x04, CVI_HDMI_SINK_CAPABILITY)
#define CVI_HDMI_SET_ATTR _IOW('H', 0x05, CVI_HDMI_ATTR)
#define CVI_HDMI_GET_ATTR _IOR('H', 0x06, CVI_HDMI_ATTR)
#define CVI_HDMI_START _IO('H', 0x07)
#define CVI_HDMI_STOP _IO('H', 0x08)
#define CVI_HDMI_FORCE_GET_EDID _IOWR('H', 0x9, CVI_HDMI_EDID)
#define CVI_HDMI_GET_EVENT_ID _IOR('H', 0xa, CVI_U32)
#define CVI_HDMI_UNREGISTER_CALLBACK _IO('H', 0xb)
#define CVI_HDMI_SET_INFOFRAME _IOWR('H', 0x0c, CVI_HDMI_INFOFRAME )
#define CVI_HDMI_GET_INFOFRAME _IOWR('H', 0x0d, CVI_HDMI_INFOFRAME)
#define CVI_HDMI_SET_HW_SPEC _IOW('H', 0x0e, CVI_HDMI_HW_SPEC)
#define CVI_HDMI_GET_HW_SPEC _IOW('H', 0x0f, CVI_HDMI_HW_SPEC)
#define CVI_HDMI_SET_AVMUTE _IOW('H', 0xa0, CVI_BOOL)
#define CVI_HDMI_SET_MOD_PARAM _IO('H', 0xa1)
#define CVI_HDMI_GET_MOD_PARAM _IO('H', 0xa2)
#define CVI_HDMI_SET_AUDIO_MUTE _IOW('H', 0xa3, CVI_BOOL)


#endif /* _U_HDMI_UAPI_H_ */
