/*
 * BM JPEG/MJPEG encoder
 * Copyright (c) 2018-2019 Solan Shang <shulin.shang@bitmain.com>
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

/**
 * @file
 * BM JPEG/MJPEG encoder
 *
 * @author Solan Shang <shulin.shang@bitmain.com>
 *
 * Unimplemented:
 *   - XXX
 */

#ifndef _BM_JPEG_ENC_H_
#define _BM_JPEG_ENC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <libavformat/avformat.h>
#include <libavutil/avutil.h>

typedef struct _BMJpegEncContext{
    const AVClass *avclass;
    AVCodecContext *avctx;
} BMJpegEncContext;

#ifdef __cplusplus
}
#endif

#endif /* _BM_JPEG_ENC_H_ */
