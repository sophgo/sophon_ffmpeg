/*
 * BM JPEG/MJPEG codec
 * Copyright (c) 2018-2020 Solan Shang <shulin.shang@bitmain.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _BM_JPEG_COMMON_H_
#define _BM_JPEG_COMMON_H_

static void logging_func(BmJpuLogLevel level, char const *file, int const line, char const *fn, const char *format, ...)
{
    va_list args;

    char const *lvlstr = "";
    switch (level) {
    case BM_JPU_LOG_LEVEL_ERROR:   lvlstr = "ERROR";   break;
    case BM_JPU_LOG_LEVEL_WARNING: lvlstr = "WARNING"; break;
    case BM_JPU_LOG_LEVEL_INFO:    lvlstr = "info";    break;
    case BM_JPU_LOG_LEVEL_DEBUG:   lvlstr = "debug";   break;
    case BM_JPU_LOG_LEVEL_TRACE:   lvlstr = "trace";   break;
    case BM_JPU_LOG_LEVEL_LOG:     lvlstr = "log";     break;
    default: break;
    }

    fprintf(stderr, "%s:%d (%s)   %s: ", file, line, fn, lvlstr);

    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    fprintf(stderr, "\n");
}

static void bmjpu_setup_logging(void)
{
    BmJpuLogLevel level = BM_JPU_LOG_LEVEL_ERROR;

    int av_log_level = av_log_get_level();
    switch (av_log_level) {
    case AV_LOG_QUIET:
    case AV_LOG_PANIC:
    case AV_LOG_FATAL:
    case AV_LOG_ERROR:
        level = BM_JPU_LOG_LEVEL_ERROR;
        break;
    case AV_LOG_WARNING:
        level = BM_JPU_LOG_LEVEL_WARNING;
        break;
    case AV_LOG_INFO:
    case AV_LOG_VERBOSE:
        level = BM_JPU_LOG_LEVEL_INFO;
        break;
    case AV_LOG_DEBUG:
        level = BM_JPU_LOG_LEVEL_DEBUG;
        break;
    case AV_LOG_TRACE:
        level = BM_JPU_LOG_LEVEL_TRACE;
        break;
    default:
        level = BM_JPU_LOG_LEVEL_ERROR;
        break;
    }

    av_log(NULL, AV_LOG_DEBUG, "av_log_level: %d\n", av_log_level);
    av_log(NULL, AV_LOG_DEBUG, "bmjpuapi logging threshold: %d\n", level);

    bm_jpu_set_logging_threshold(level);
    // bm_jpu_set_logging_function(logging_func);
}

#endif /* _BM_JPEG_COMMON_H_ */

