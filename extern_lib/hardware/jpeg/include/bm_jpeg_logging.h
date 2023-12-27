#ifndef __BM_JPEG_LOGGING_H__
#define __BM_JPEG_LOGGING_H__

#include "bm_jpeg_interface.h"

/* Log levels. */
typedef enum
{
    BM_JPU_LOG_LEVEL_ERROR   = 0,
    BM_JPU_LOG_LEVEL_WARNING = 1,
    BM_JPU_LOG_LEVEL_INFO    = 2,
    BM_JPU_LOG_LEVEL_DEBUG   = 3,
    BM_JPU_LOG_LEVEL_LOG     = 4,
    BM_JPU_LOG_LEVEL_TRACE   = 5
} BmJpuLogLevel;

extern BmJpuLogLevel bm_jpu_cur_log_level_threshold;

#if defined (__cplusplus)
extern "C" {
#endif

void bm_jpu_set_logging_threshold(BmJpuLogLevel threshold);
void logging_fn(BmJpuLogLevel level, char const *file, int const line, char const *fn, const char *format, ...);

#define BM_JPU_ERROR_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_ERROR)   { logging_fn(BM_JPU_LOG_LEVEL_ERROR,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_WARNING_FULL(FILE_, LINE_, FUNCTION_, ...) do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_WARNING) { logging_fn(BM_JPU_LOG_LEVEL_WARNING, FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_INFO_FULL(FILE_, LINE_, FUNCTION_, ...)    do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_INFO)    { logging_fn(BM_JPU_LOG_LEVEL_INFO,    FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_DEBUG_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_DEBUG)   { logging_fn(BM_JPU_LOG_LEVEL_DEBUG,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_LOG_FULL(FILE_, LINE_, FUNCTION_, ...)     do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_LOG)     { logging_fn(BM_JPU_LOG_LEVEL_LOG,     FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)
#define BM_JPU_TRACE_FULL(FILE_, LINE_, FUNCTION_, ...)   do { if (bm_jpu_cur_log_level_threshold >= BM_JPU_LOG_LEVEL_TRACE)   { logging_fn(BM_JPU_LOG_LEVEL_TRACE,   FILE_, LINE_, FUNCTION_, __VA_ARGS__); } } while(0)


#define BM_JPU_ERROR(...)    BM_JPU_ERROR_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_WARNING(...)  BM_JPU_WARNING_FULL(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_INFO(...)     BM_JPU_INFO_FULL   (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_DEBUG(...)    BM_JPU_DEBUG_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_LOG(...)      BM_JPU_LOG_FULL    (__FILE__, __LINE__, __func__, __VA_ARGS__)
#define BM_JPU_TRACE(...)    BM_JPU_TRACE_FULL  (__FILE__, __LINE__, __func__, __VA_ARGS__)

char const *bm_jpu_color_format_string(BmJpuColorFormat color_format);
char const *bm_jpu_dec_error_string(BmJpuDecReturnCodes code);
char const *bm_jpu_enc_error_string(BmJpuEncReturnCodes code);

#if defined (__cplusplus)
}
#endif

#endif /* __BM_JPEG_LOGGING_H__ */