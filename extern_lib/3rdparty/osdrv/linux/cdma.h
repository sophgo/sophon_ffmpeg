#ifndef __CDMA_H__
#define __CDMA_H__

enum CDMA_DATD_TYPE {
	CDMA_DATA_TYPE_8BIT = 0,
	CDMA_DATA_TYPE_16BIT,
};

struct cdma_1d_param {
	u64 src_addr;
	u64 dst_addr;
	u32 len;
	enum CDMA_DATD_TYPE data_type;
};

struct cdma_2d_param {
	u64 src_addr;
	u64 dst_addr;
	u16 width;
	u16 height;
	u16 src_stride;
	u16 dst_stride;
	enum CDMA_DATD_TYPE data_type;
	bool isFixed;
	u16 fixed_value;
};

void cdma_set_base_addr(void *cdma_base);
void cdma_irq_handler(void);

int cvi_cdma_copy1d(struct cdma_1d_param *param);
int cvi_cdma_copy2d(struct cdma_2d_param *param);

#endif
