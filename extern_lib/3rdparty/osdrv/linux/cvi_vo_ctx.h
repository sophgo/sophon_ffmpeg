#ifndef __U_CVI_VO_CTX_H__
#define __U_CVI_VO_CTX_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include <linux/cvi_vip.h>
#include <linux/cvi_defines.h>
#include <linux/cvi_comm_vo.h>
#include <linux/cvi_comm_region.h>

#define VO_SHARE_MEM_SIZE           (0x2000)
struct cvi_vo_ctx {
	CVI_BOOL is_dev_enable[VO_MAX_DEV_NUM];
	CVI_BOOL is_layer_enable[VO_MAX_LAYER_NUM];
	CVI_BOOL is_chn_enable[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];

	// dev
	VO_PUB_ATTR_S stPubAttr[VO_MAX_DEV_NUM];

	// layer
	VO_VIDEO_LAYER_ATTR_S stLayerAttr[VO_MAX_LAYER_NUM];
	CVI_U32 u32DisBufLen[VO_MAX_LAYER_NUM];
	CVI_S32 proc_amp[VO_MAX_LAYER_NUM][PROC_AMP_MAX];
	VO_LAYER_BIND_S stLayerBind[VO_MAX_LAYER_NUM];
	// struct vb_jobs_t vb_jobs[VO_MAX_LAYER_NUM];

	// chn
	VO_CHN_ATTR_S stChnAttr[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];
	ROTATION_E enRotation[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];
	RGN_HANDLE rgn_handle[VO_MAX_LAYER_NUM][RGN_MAX_NUM_VO];
	RGN_HANDLE rgn_coverEx_handle[VO_MAX_LAYER_NUM][RGN_COVEREX_MAX_NUM];
	CVI_U64 u64DisplayPts[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];
	CVI_U64 u64PreDonePts[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];

	// for calculating chn frame rate
	VO_CHN_STATUS_S chnStatus[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];

	struct cvi_rgn_cfg rgn_cfg[VO_MAX_LAYER_NUM];
	struct cvi_rgn_coverex_cfg rgn_coverex_cfg[VO_MAX_LAYER_NUM];

	struct {
		CVI_U64 paddr;
		CVI_VOID *vaddr;
	} mesh[VO_MAX_LAYER_NUM];
	//pthread_t thread[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];
	//struct task_struct *thread[VO_MAX_LAYER_NUM][VO_MAX_CHN_NUM];
	CVI_BOOL show[VO_MAX_LAYER_NUM];
	CVI_BOOL pause[VO_MAX_LAYER_NUM];
	CVI_BOOL clearchnbuf[VO_MAX_LAYER_NUM];
	CVI_BOOL fb_on_vpss[VO_MAX_LAYER_NUM];
	struct{
		CVI_U32 left;
		CVI_U32 top;
		CVI_U32 width;
		CVI_U32 height;
	} rect_crop[VO_MAX_LAYER_NUM];
};

#ifdef __cplusplus
}
#endif

#endif /* __U_CVI_VO_CTX_H__ */
