#ifndef _BM_JPEG_IOCTL_H_
#define _BM_JPEG_IOCTL_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "linux/cvi_vc_drv_ioctl.h"
#include "cvi_structs.h"
#include "linux/cvi_comm_vb.h"


void bmjpeg_devm_open(void);
void bmjpeg_devm_close(void);
void* bmjpeg_devm_map(uint64_t phys_addr, size_t len);
void bmjpeg_devm_unmap(void *virt_addr, size_t len);
void* bmjpeg_ioctl_mmap(int chn_fd, unsigned long long phys_addr, size_t len);
void bmjpeg_ioctl_munmap(uint8_t *virt_addr, size_t len);
int bmjpeg_ioctl_get_chn(int chn_fd, int *chn_id);
int bmjpeg_ioctl_set_chn(int chn_fd, int *chn_id);

int bmjpeg_dec_ioctl_create_chn(int chn_fd, VDEC_CHN_ATTR_S *pstAttr);
int bmjpeg_dec_ioctl_destroy_chn(int chn_fd);
int bmjpeg_dec_ioctl_get_chn_attr(int chn_fd, VDEC_CHN_ATTR_S *pstAttr);
int bmjpeg_dec_ioctl_set_chn_attr(int chn_fd, VDEC_CHN_ATTR_S *pstAttr);
int bmjpeg_dec_ioctl_get_chn_param(int chn_fd, VDEC_CHN_PARAM_S *pstParam);
int bmjpeg_dec_ioctl_set_chn_param(int chn_fd, VDEC_CHN_PARAM_S *pstParam);
int bmjpeg_dec_ioctl_start_recv_stream(int chn_fd);
int bmjpeg_dec_ioctl_stop_recv_stream(int chn_fd);
int bmjpeg_dec_ioctl_send_stream(int chn_fd, VDEC_STREAM_EX_S *pstStreamEx);
int bmjpeg_dec_ioctl_get_frame(int chn_fd, VIDEO_FRAME_INFO_EX_S *pstFrameInfoEx);
int bmjpeg_dec_ioctl_release_frame(int chn_fd, const VIDEO_FRAME_INFO_S *pstFrameInfo);

int bmjpeg_enc_ioctl_create_chn(int chn_fd, VENC_CHN_ATTR_S *pstAttr);
int bmjpeg_enc_ioctl_destroy_chn(int chn_fd);
int bmjpeg_enc_ioctl_get_chn_attr(int chn_fd, VENC_CHN_ATTR_S *pstAttr);
int bmjpeg_enc_ioctl_start_recv_frame(int chn_fd, VENC_RECV_PIC_PARAM_S *pstRecvParam);
int bmjpeg_enc_ioctl_stop_recv_frame(int chn_fd);
int bmjpeg_enc_ioctl_send_frame(int chn_fd, VIDEO_FRAME_INFO_EX_S *pstFrameEx);
int bmjpeg_enc_ioctl_get_stream(int chn_fd, VENC_STREAM_EX_S *pstStreamEx);
int bmjpeg_enc_ioctl_query_status(int chn_fd, VENC_CHN_STATUS_S *pstStatus);
int bmjpeg_enc_ioctl_release_stream(int chn_fd, VENC_STREAM_S *pstStream);
int bmjpeg_enc_ioctl_reset_chn(int chn_fd);

#endif /* _BM_JPEG_IOCTL_H */