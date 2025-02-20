#ifndef TEST_MISC_H
#define TEST_MISC_H

#include <stdint.h>

enum bm_data_type_t {
    DT_INT8   = (0 << 1) | 1,
    DT_UINT8  = (0 << 1) | 0,
    DT_INT16  = (3 << 1) | 1,
    DT_UINT16 = (3 << 1) | 0,
    DT_FP16   = (1 << 1) | 1,
    DT_BFP16  = (5 << 1) | 1,
    DT_INT32  = (4 << 1) | 1,
    DT_UINT32 = (4 << 1) | 0,
    DT_FP32   = (2 << 1) | 1
};

int dtype_size(enum bm_data_type_t data_type);

union conv_ {
    float f;
    uint32_t i;
};

typedef struct {
    unsigned short  manti : 10;
    unsigned short  exp : 5;
    unsigned short  sign : 1;
} fp16;

typedef struct {
    unsigned int manti : 23;
    unsigned int exp : 8;
    unsigned int sign : 1;
} fp32;

union fp16_data {
    unsigned short idata;
    fp16 ndata;
};

union fp32_data {
    unsigned int idata;
    int idataSign;
    float fdata;
    fp32 ndata;
};

float fp16tofp32(fp16 h);
fp16 fp32tofp16 (float A, int round_method);
int array_cmp_float(float *p_exp, float *p_got, int len, float delta);
int array_cmp_fix8b(void *p_exp, void *p_got, int sign, int len, int delta);
int array_cmp_fix16b(void *p_exp, void *p_got, int sign, int len, int delta);
#endif
