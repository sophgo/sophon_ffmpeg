#ifndef _BM_JPEG_IOCTL_H_
#define _BM_JPEG_IOCTL_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "linux/vc_uapi.h"
#include "linux/comm_vb.h"
#include <linux/comm_vdec.h>
#include <linux/comm_venc.h>
typedef struct _vdec_stream_ex_s {
    vdec_stream_s *pstStream;
    int s32MilliSec;
} vdec_stream_ex_s;

typedef struct _venc_stream_ex_s {
    venc_stream_s *pstStream;
    int s32MilliSec;
} venc_stream_ex_s;

typedef struct _video_frame_info_ex_s {
    video_frame_info_s *pstFrame;
    int s32MilliSec;
} video_frame_info_ex_s;
void bmjpeg_devm_open(void);
void bmjpeg_devm_close(void);
void* bmjpeg_devm_map(uint64_t phys_addr, size_t len);
void bmjpeg_devm_unmap(void *virt_addr, size_t len);
void* bmjpeg_ioctl_mmap(int chn_fd, unsigned long long phys_addr, size_t len);
void bmjpeg_ioctl_munmap(uint8_t *virt_addr, size_t len);
int bmjpeg_ioctl_get_chn(int chn_fd, int *chn_id);
int bmjpeg_ioctl_set_chn(int chn_fd, int *chn_id);

int bmjpeg_dec_ioctl_create_chn(int chn_fd, vdec_chn_attr_s *pstAttr);
int bmjpeg_dec_ioctl_destroy_chn(int chn_fd);
int bmjpeg_dec_ioctl_get_chn_attr(int chn_fd, vdec_chn_attr_s *pstAttr);
int bmjpeg_dec_ioctl_set_chn_attr(int chn_fd, vdec_chn_attr_s *pstAttr);
int bmjpeg_dec_ioctl_get_chn_param(int chn_fd, vdec_chn_param_s *pstParam);
int bmjpeg_dec_ioctl_set_chn_param(int chn_fd, vdec_chn_param_s *pstParam);
int bmjpeg_dec_ioctl_start_recv_stream(int chn_fd);
int bmjpeg_dec_ioctl_stop_recv_stream(int chn_fd);
int bmjpeg_dec_ioctl_send_stream(int chn_fd, vdec_stream_ex_s *pstStreamEx);
int bmjpeg_dec_ioctl_get_frame(int chn_fd,  video_frame_info_ex_s *pstFrameInfoEx);
int bmjpeg_dec_ioctl_release_frame(int chn_fd, const video_frame_info_s *pstFrameInfo);

int bmjpeg_enc_ioctl_create_chn(int chn_fd, venc_chn_attr_s *pstAttr);
int bmjpeg_enc_ioctl_destroy_chn(int chn_fd);
int bmjpeg_enc_ioctl_get_chn_attr(int chn_fd, venc_chn_attr_s *pstAttr);
int bmjpeg_enc_ioctl_start_recv_frame(int chn_fd, venc_recv_pic_param_s *pstRecvParam);
int bmjpeg_enc_ioctl_stop_recv_frame(int chn_fd);
int bmjpeg_enc_ioctl_send_frame(int chn_fd, video_frame_info_ex_s *pstFrameEx);
int bmjpeg_enc_ioctl_get_stream(int chn_fd, venc_stream_ex_s *pstStreamEx);
int bmjpeg_enc_ioctl_query_status(int chn_fd, venc_chn_status_s *pstStatus);
int bmjpeg_enc_ioctl_release_stream(int chn_fd, venc_stream_s *pstStream);
int bmjpeg_enc_ioctl_reset_chn(int chn_fd);

#endif /* _BM_JPEG_IOCTL_H */