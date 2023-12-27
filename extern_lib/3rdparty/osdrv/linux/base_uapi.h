#ifndef __BASE_UAPI_H__
#define __BASE_UAPI_H__


#include <linux/version.h>
#include <linux/types.h>
#include <linux/cvi_comm_sys.h>

#define MAX_ION_BUFFER_NAME 32

#define VB_POOL_NAME_LEN        (32)
#define VB_COMM_POOL_MAX_CNT    (16)

#define BASE_LOG_LEVEL_OFFSET       (0x0)
#define LOG_LEVEL_RSV_SIZE          (sizeof(CVI_S32) * CVI_ID_BUTT)

#define BASE_VERSION_INFO_OFFSET    (BASE_LOG_LEVEL_OFFSET + LOG_LEVEL_RSV_SIZE)
#define VERSION_INFO_RSV_SIZE       (sizeof(MMF_VERSION_S))

#define BASE_SHARE_MEM_SIZE         ALIGN(BASE_VERSION_INFO_OFFSET + VERSION_INFO_RSV_SIZE, 0x1000)

enum VB_IOCTL {
	VB_IOCTL_SET_CONFIG,
	VB_IOCTL_GET_CONFIG,
	VB_IOCTL_INIT,
	VB_IOCTL_EXIT,
	VB_IOCTL_CREATE_POOL,
	VB_IOCTL_DESTROY_POOL,
	VB_IOCTL_PHYS_TO_HANDLE,
	VB_IOCTL_GET_BLK_INFO,
	VB_IOCTL_GET_POOL_CFG,
	VB_IOCTL_GET_BLOCK,
	VB_IOCTL_RELEASE_BLOCK,
	VB_IOCTL_GET_POOL_MAX_CNT,
	VB_IOCTL_PRINT_POOL,
	VB_IOCTL_UNIT_TEST,
	VB_IOCTL_MAX,
};

/*
 * blk_size: size of blk in the pool.
 * blk_cnt: number of blk in the pool.
 * remap_mode: remap mode.
 * name: pool name
 * pool_id: pool id
 * mem_base: phy start addr of this pool
 */
struct cvi_vb_pool_cfg {
	__u32 blk_size;
	__u32 blk_cnt;
	__u8 remap_mode;
	char pool_name[VB_POOL_NAME_LEN];
	__u32 pool_id;
	__u64 mem_base;
};

/*
 * comm_pool_cnt: number of common pools used.
 * comm_pool: pool cfg for the pools.
 */
struct cvi_vb_cfg {
	__u32 comm_pool_cnt;
	struct cvi_vb_pool_cfg comm_pool[VB_COMM_POOL_MAX_CNT];
};

struct cvi_vb_blk_cfg {
	__u32 pool_id;
	__u32 blk_size;
	__u64 blk;
};

struct cvi_vb_blk_info {
	__u64 blk;
	__u32 pool_id;
	__u64 phy_addr;
	__u32 usr_cnt;
};

struct vb_ext_control {
	__u32 id;
	__u32 reserved[1];
	union {
		__s32 value;
		__s64 value64;
		void *ptr;
	};
} __attribute__ ((packed));

struct sys_cache_op {
	void *addr_v;
	__u64 addr_p;
	__u64 size;
	__s32 dma_fd;
};

struct sys_ion_data {
	__u32 size;
	__u32 cached;
	__u64 addr_p;
	__u8 name[MAX_ION_BUFFER_NAME];
};

struct sys_bind_cfg {
	__u32 is_bind;
	__u32 get_by_src;
	MMF_CHN_S mmf_chn_src;
	MMF_CHN_S mmf_chn_dst;
	MMF_BIND_DEST_S bind_dst;
};


#define IOCTL_BASE_MAGIC	'b'
#define BASE_VB_CMD		    _IOWR(IOCTL_BASE_MAGIC, 0x01, struct vb_ext_control)
#define BASE_SET_BINDCFG	_IOW(IOCTL_BASE_MAGIC, 0x02, struct sys_bind_cfg)
#define BASE_GET_BINDCFG	_IOWR(IOCTL_BASE_MAGIC, 0x03, struct sys_bind_cfg)
#define BASE_ION_ALLOC		_IOWR(IOCTL_BASE_MAGIC, 0x04, struct sys_ion_data)
#define BASE_ION_FREE		_IOW(IOCTL_BASE_MAGIC, 0x05, struct sys_ion_data)
#define BASE_CACHE_INVLD	_IOW(IOCTL_BASE_MAGIC, 0x06, struct sys_cache_op)
#define BASE_CACHE_FLUSH	_IOW(IOCTL_BASE_MAGIC, 0x07, struct sys_cache_op)


#endif
