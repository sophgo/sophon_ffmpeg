#ifndef BMCV_INTERNAL_H
#define BMCV_INTERNAL_H
#include "bmcv_api_ext_c.h"
#include "md5.h"
#include "bmlib_runtime.h"
#include "bmlib_interface.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

#ifndef USING_CMODEL
// #define _FPGA

#include "bm_jpeg_interface.h"

#endif

// #define BMCV_VERSION "1.1.0"

#define __ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))

#ifdef __linux__
#define ALIGN(x, a) __ALIGN_MASK(x, (__typeof__(x))(a)-1)
#else
#define ALIGN(x, a) __ALIGN_MASK(x, (int)(a)-1)
#endif

#ifdef NO_PRINTF_IN_ASSERT
#define ASSERT_INFO(_cond, fmt, ...) \
  do {                               \
    if (!(_cond)) {                  \
      hang(-1);                      \
    }                                \
  } while (0)
#else
#define ASSERT_INFO(_cond, fmt, ...)                                           \
  do {                                                                         \
    if (!(_cond)) {                                                            \
      printf("ASSERT %s: %s: %d: %s\n", __FILE__, __func__, __LINE__, #_cond); \
      printf("ASSERT info: " fmt "\n", ##__VA_ARGS__);                         \
    }                                                                          \
  } while (0)
#endif

#define ASSERT(_cond) ASSERT_INFO(_cond, "none.")

#define ASSERT_RANGE(val, min, max) \
  ASSERT_INFO((val) >= (min) && (val) <= (max), #val "=%d must be in [%d, %d]", (val), (min), (max))

void bmcv_err_log_internal(char *log_buf, size_t string_sz, const char *frmt, ...);
#ifdef __linux__
#define BMCV_ERR_LOG(frmt, args...)                                            \
    do {                                                                       \
        char log_buffer[MAX_BMCV_LOG_BUFFER_SIZE] = {0};                       \
        int  string_sz                            = 0;                         \
        snprintf(log_buffer,                                                   \
                 MAX_BMCV_LOG_BUFFER_SIZE,                                     \
                 " [MESSAGE FROM %s: %s: %d]: ",                               \
                 filename(__FILE__),                                           \
                 __func__,                                                     \
                 __LINE__);                                                    \
        string_sz = strlen(log_buffer);                                        \
        bmcv_err_log_internal(log_buffer, string_sz, frmt, ##args);            \
    } while (0)
#else
#define BMCV_ERR_LOG(frmt)                                                     \
    do {                                                                       \
        char log_buffer[MAX_BMCV_LOG_BUFFER_SIZE] = {0};                       \
        size_t  string_sz                         = 0;                         \
        snprintf(log_buffer,                                                   \
                 MAX_BMCV_LOG_BUFFER_SIZE,                                     \
                 " [MESSAGE FROM %s: %s: %d]: ",                               \
                 filename(__FILE__),                                           \
                 __func__,                                                     \
                 __LINE__);                                                    \
        string_sz = strlen(log_buffer);                                        \
        bmcv_err_log_internal(log_buffer, string_sz, frmt);                    \
    } while (0)
#endif
#if defined(__cplusplus)
#endif

/*
 * bmcv api for using internal.
 */

typedef unsigned char        u8;
typedef unsigned short       u16;
typedef unsigned int         u32;
typedef unsigned long long   u64;

typedef signed char          s8;
typedef signed short         s16;
typedef signed int           s32;
typedef signed long long int s64;

#define MIN_PROPOSAL_NUM (1)
#define BMCV_LOG_TAG "BMCV"
#define MAX_BMCV_LOG_BUFFER_SIZE (256)
#define MAX_bm_image_CHANNEL 4
#define BM1686 0x1686a200
#define COLOR_SPACE_YUV             0
#define COLOR_SPACE_RGB             1
#define COLOR_SPACE_HSV             2
#define COLOR_SPACE_RGBY            3
#define COLOR_NOT_DEF               4

#define filename(x) (strrchr(x, '/') ? strrchr(x, '/') + 1 : x)

#define BM_MIN(x, y) (((x)) < ((y)) ? (x) : (y))
#define BM_MAX(x, y) (((x)) > ((y)) ? (x) : (y))

#define VPP_MAX_SCALING_RATIO (32)
#define MAX_BATCH_SIZE (32)
#define MOSAIC_SIZE 8
#define VPPALIGN(x, mask)  (((x) + ((mask)-1)) & ~((mask)-1))

#define BM_API_ID_CV_CORRECT_LAYOUT 506

typedef struct
{
    bool flag;
    bm_device_mem_t bm_device_mem;
}sg_device_mem_st;

typedef struct {
    int   index;
    float val;
} __attribute__((packed)) sort_t;

typedef struct bm_mem_desc bm_device_mem_t;
typedef struct bm_mem_desc bm_system_mem_t;

// typedef enum {
//   BM_SUCCESS = 0,
//   BM_ERR_DEVNOTREADY = 1, /* Device not ready yet */
//   BM_ERR_FAILURE = 2,     /* General failure */
//   BM_ERR_TIMEOUT = 3,     /* Timeout */
//   BM_ERR_PARAM = 4,       /* Parameters invalid */
//   BM_ERR_NOMEM = 5,       /* Not enough memory */
//   BM_ERR_DATA = 6,        /* Data error */
//   BM_ERR_BUSY = 7,        /* Busy */
//   BM_ERR_NOFEATURE = 8,   /* Not supported yet */
//   BM_NOT_SUPPORTED = 9
// } bm_status_t;

typedef enum {
  STORAGE_MODE_1N_FP32    = 0,
  STORAGE_MODE_1N_INT8    = 1,
  STORAGE_MODE_1N_INT16   = 2,
  STORAGE_MODE_2N_INT16   = 3,
  STORAGE_MODE_4N_INT8    = 4,
  STORAGE_MODE_2IC_FP32   = 5,  // special for 2IC weight
  STORAGE_MODE_4N_4IC_4OC = 6,
  STORAGE_MODE_4N_INT16   = 7,
  STORAGE_MODE_UNINITILIZED,
  STORAGE_MODE_END
} TENSOR_STORAGE_MODE;

typedef enum {
    MAX_SIZE_MODE = 0,
    MIN_SIZE_MODE,
    RAND_SIZE_MODE,
    MAX_RAND_MODE
} rand_mode_e;

typedef enum {
    CONVERT_1N_TO_1N = 0,
    CONVERT_4N_TO_4N,
    CONVERT_4N_TO_1N
} convert_storage_mode_e;

typedef enum {
    UINT8_C1 = 0,
    UINT8_C3,
    INT8_C1,
    INT8_C3,
    FLOAT32_C1,
    FLOAT32_C3,
    MAX_COLOR_TYPE
} cv_color_e;

typedef struct plane_layout{
    int N;
    int C;
    int H;
    int W;
    int data_size;
    int pitch_stride;
    int channel_stride;
    int batch_stride;
    #ifdef __linux__
    unsigned long long size;
    #else
    int size;
    #endif
} plane_layout;


typedef struct bm_image_private{
    bm_handle_t     handle;
    bm_device_mem_t data[MAX_bm_image_CHANNEL];
    plane_layout    memory_layout[MAX_bm_image_CHANNEL];
    int             plane_num;
    int             internal_alloc_plane;
    pthread_mutex_t memory_lock;
    bool            attached;
    bool            data_owned;
    bool            default_stride;
#ifndef USING_CMODEL
    BmJpuJPEGDecoder *decoder;
#endif
} bm_image_private;


typedef struct bm_api_width_align {
  u64 S_global_offset;
  u64 D_global_offset;
  int N;
  int C;
  int H;
  int W;
  int src_n_stride;
  int src_c_stride;
  int src_h_stride;
  int dst_n_stride;
  int dst_c_stride;
  int dst_h_stride;
  int data_size;
} bm_api_cv_width_align_t;

plane_layout set_plane_layout(int n, int c, int h, int w, int Dsize);

plane_layout align_width(plane_layout src, int align);

plane_layout align_height(plane_layout src, int align);

plane_layout align_channel(plane_layout  src,
                           int           pitch_align,
                           int           height_align);

plane_layout stride_width(plane_layout src, int stride);

u8 is_full_image(bm_image_format_ext image_format);
u8 is_yuv420_image(bm_image_format_ext image_format);

bm_status_t  update_memory_layout(bm_handle_t     handle,
                                  bm_device_mem_t src,
                                  plane_layout    src_layout,
                                  bm_device_mem_t dst,
                                  plane_layout    dst_layout);

DECL_EXPORT bm_status_t bm_send_api(
  bm_handle_t  handle,
  int api_id,
  const u8     *api,
  u32          size);

DECL_EXPORT bm_status_t bm_sync_api(bm_handle_t handle);

bm_status_t bm_get_mem_fd(int* fd);
bm_status_t bm_get_vpss_fd(int* fd);
bm_status_t bm_destroy_vpss_fd(void);
bm_status_t bm_destroy_mem_fd(void);
bm_status_t bm_get_dpu_fd(int* fd);
bm_status_t bm_destroy_dpu_fd(void);
bm_status_t bm_get_ldc_fd(int* fd);
bm_status_t bm_destroy_ldc_fd(void);
bm_status_t bm_get_dwa_fd(int* fd);
bm_status_t bm_destroy_dwa_fd(void);
bm_status_t sg_image_alloc_dev_mem(bm_image image, int heap_id);
bm_status_t sg_malloc_device_mem(bm_handle_t handle, sg_device_mem_st *pmem, unsigned int size);
void sg_free_device_mem(bm_handle_t handle, sg_device_mem_st mem);

#ifdef _FPGA
bm_status_t bm_memcpy_s2d_fpga(bm_handle_t handle, bm_device_mem_t dst, void *src);
bm_status_t bm_memcpy_d2s_fpga(bm_handle_t handle, void *dst, bm_device_mem_t src);
#endif

#endif
