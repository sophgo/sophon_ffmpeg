#ifndef __CVI_STRUCTS_H__
#define __CVI_STRUCTS_H__

#include <linux/cvi_comm_vdec.h>
#include <linux/cvi_comm_venc.h>

typedef struct _VDEC_STREAM_EX_S {
    VDEC_STREAM_S *pstStream;
    CVI_S32 s32MilliSec;
} VDEC_STREAM_EX_S;

typedef struct _VENC_STREAM_EX_S {
    VENC_STREAM_S *pstStream;
    CVI_S32 s32MilliSec;
} VENC_STREAM_EX_S;

typedef struct _VIDEO_FRAME_INFO_EX_S {
    VIDEO_FRAME_INFO_S *pstFrame;
    CVI_S32 s32MilliSec;
} VIDEO_FRAME_INFO_EX_S;

#endif /* __CVI_STRUCTS_H__ */