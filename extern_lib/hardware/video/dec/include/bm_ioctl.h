/*****************************************************************************
 *
 *    Copyright (C) 2022 Sophgo Technologies Inc.  All rights reserved.
 *
 *    bmvid is licensed under the 2-Clause BSD License except for the
 *    third-party components.
 *
 *****************************************************************************/
/* This library provides a high-level interface for controlling the BitMain
 * Sophon VPU en/decoder.
 */

#ifndef _BM_VPUDEC_IOCTL_H_
#define _BM_VPUDEC_IOCTL_H_


#include <stdint.h>
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>

typedef unsigned int VB_POOL;

#include <linux/defines.h>
#include <linux/common.h>
#include <linux/comm_rc.h>
#include <linux/comm_video.h>
#include <linux/comm_vdec.h>
#include <linux/vc_uapi.h>
#include "bm_vpudec_interface.h"


#ifndef BM_UNUSED_PARAMS
#define BM_UNUSED_PARAMS(x) ((void)(x))
#endif

typedef struct _video_frame_info_ex_s {
	video_frame_info_s *pstFrame;
	int s32MilliSec;
} video_frame_info_ex_s;

typedef struct _vdec_stream_ex_s  {
	vdec_stream_s *pstStream;
	int s32MilliSec;
} vdec_stream_ex_s;

extern BmVpuDecLogLevel bm_vpu_log_level_threshold;

void bm_vpu_set_logging_threshold(BmVpuDecLogLevel threshold);
void logging_fn(BmVpuDecLogLevel level, char const *file, int const line, char const *fn, const char *format, ...);

#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define BM_VPU_ERROR_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_ERROR)   { logging_fn(BM_VPU_LOG_LEVEL_ERROR,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_VPU_WARNING_FULL(FILE_, LINE_, FUNCTION_, ...) do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_WARNING) { logging_fn(BM_VPU_LOG_LEVEL_WARNING, FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_VPU_INFO_FULL(FILE_, LINE_, FUNCTION_, ...)    do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_INFO)    { logging_fn(BM_VPU_LOG_LEVEL_INFO,    FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_VPU_DEBUG_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_DEBUG)   { logging_fn(BM_VPU_LOG_LEVEL_DEBUG,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_VPU_LOG_FULL(FILE_, LINE_, FUNCTION_, ...)     do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_LOG)     { logging_fn(BM_VPU_LOG_LEVEL_LOG,     FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_VPU_TRACE_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_vpu_log_level_threshold >= BM_VPU_LOG_LEVEL_TRACE)   { logging_fn(BM_VPU_LOG_LEVEL_TRACE,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)

#define BMVPU_DEC_ERROR(...)    BM_VPU_ERROR_FULL  (FILENAME, __LINE__, __func__, __VA_ARGS__)
#define BMVPU_DEC_WARNING(...)  BM_VPU_WARNING_FULL(FILENAME, __LINE__, __func__, __VA_ARGS__)
#define BMVPU_DEC_INFO(...)     BM_VPU_INFO_FULL   (FILENAME, __LINE__, __func__, __VA_ARGS__)
#define BMVPU_DEC_DEBUG(...)    BM_VPU_DEBUG_FULL  (FILENAME, __LINE__, __func__, __VA_ARGS__)
#define BMVPU_DEC_LOG(...)      BM_VPU_LOG_FULL    (FILENAME, __LINE__, __func__, __VA_ARGS__)
#define BMVPU_DEC_TRACE(...)    BM_VPU_TRACE_FULL  (FILENAME, __LINE__, __func__, __VA_ARGS__)

int bmdec_chn_open(char* dev_name, int soc_idx);
int bmdec_chn_close(int soc_idx);

int bmdec_ioctl_get_chn(int chn_fd, int *is_jpu, int *chn_id);
int bmdec_ioctl_set_chn(int chn_fd, int* chn_id);
int bmdec_ioctl_create_chn(int chn_fd, vdec_chn_attr_s* pstAttr);
int bmdec_ioctl_destory_chn(int chn_fd);
int bmdec_ioctl_start_recv_stream(int chn_fd);
int bmdec_ioctl_stop_recv_stream(int chn_fd);
int bmdec_ioctl_send_stream(int chn_fd, vdec_stream_ex_s* pstStreamEx);
int bmdec_ioctl_get_frame(int chn_fd, video_frame_info_ex_s* pstFrameInfoEx, vdec_chn_status_s* stChnStatus);
int bmdec_ioctl_release_frame(int chn_fd, const video_frame_info_s *pstFrameInfo, int size);
int bmdec_ioctl_get_chn_attr(int chn_fd, vdec_chn_attr_s *pstAttr);
int bmdec_ioctl_set_chn_attr(int chn_fd, vdec_chn_attr_s *pstAttr);
int bmdec_ioctl_query_chn_status(int chn_fd, vdec_chn_status_s *pstStatus);
int bmdec_ioctl_set_chn_param(int chn_fd, const vdec_chn_param_s *pstParam);
int bmdec_ioctl_get_chn_param(int chn_fd, vdec_chn_param_s *pstParam);

#endif /* _BM_VPUENC_IOCTL_H_ */
