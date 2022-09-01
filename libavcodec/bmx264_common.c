/*
 * H.264 encoding using the bmx264 hardware acceleration
 *
 * Copyright (C) 2020 Solan Shang <shulin.shang@bitmain.com>
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

#if defined(BM_PCIE_MODE) && defined(BM1684)

#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <sys/time.h>

#include "libavutil/mem.h"
#include "libavutil/pixfmt.h"
#include "bmx264_common.h"

#define KEYINT_MAX_INFINITE     (1<<30)

#define FAIL_IF_ERROR(cond, ...)\
do{\
    if(cond) {\
        av_log(avctx, AV_LOG_ERROR, __VA_ARGS__);\
        goto failed;\
    }\
} while(0)


static const char* bmx264_overscan_names[] = {
    "undef", "show", "crop", 0
};
static const char* bmx264_vidformat_names[] = {
    "component", "pal", "ntsc", "secam", "mac", "undef", 0
};
static const char* bmx264_fullrange_names[] = {
    "off", "on", 0
};
static const char* bmx264_colorprim_names[] = {
    "", "bt709", "undef", "", "bt470m", "bt470bg",
    "smpte170m", "smpte240m", "film",
    "bt2020", "smpte428", "smpte431", "smpte432", 0
};
static const char* bmx264_transfer_names[] = {
    "", "bt709", "undef", "", "bt470m", "bt470bg", "smpte170m", "smpte240m",
    "linear", "log100", "log316",
    "iec61966-2-4", "bt1361e", "iec61966-2-1", "bt2020-10", "bt2020-12",
    "smpte2084", "smpte428", "arib-std-b67", 0
};
static const char* bmx264_colmatrix_names[] = {
    "GBR", "bt709", "undef", "", "fcc", "bt470bg", "smpte170m", "smpte240m", "YCgCo",
    "bt2020nc", "bt2020c",
    "smpte2085", "chroma-derived-nc", "chroma-derived-c", "ICtCp", 0
};

static const char* bmx264_b_pyramid_names[] = { "none", "strict", "normal", 0 };
static const char* bmx264_direct_pred_names[] = { "none", "spatial", "temporal", "auto", 0 };
static const char* bmx264_motion_est_names[] = { "dia", "hex", "umh", "esa", "tesa", 0 };
static const char* bmx264_nal_hrd_names[] = { "none", "vbr", "cbr", 0 };

int bmx264_init(AVCodecContext* avctx, int device_idx, int width, int height)
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;
    char* bmcpu_fipfile     = getenv("BMCPU_FIPFILE");
    char* bmcpu_rambootfile = getenv("BMCPU_RAMBOOTFILE");
    char* bmx264_logfile    = getenv("BMCPU_LOGFILE");
    char* bmx264_libfile    = getenv("BMX264_LIBFILE");
    int params_size = sizeof(x264_param_t);
    int pic_size = sizeof(x264_picture_t);
    int payload_size = width*height;  // TODO
    int size = sizeof(int)+MAX_NUM_NAL*sizeof(x264_nal_t)+payload_size+8;
    struct timeval ps, pe;
    int ret = 0;

    bmx->device_idx = device_idx;

    ret = bm_dev_request(&(bmx->device_handle), device_idx);
    FAIL_IF_ERROR((ret !=0 || bmx->device_handle == NULL),
                  "failed to bm_dev_request()!\n");

    FAIL_IF_ERROR(bmcpu_fipfile==NULL, "please set env BMCPU_FIPFILE!\n");
    ret = access(bmcpu_fipfile, F_OK|R_OK);
    FAIL_IF_ERROR(ret<0, "please check BMCPU_FIPFILE(%s)!\n", bmcpu_fipfile);

    FAIL_IF_ERROR(bmcpu_rambootfile==NULL, "please set env BMCPU_RAMBOOTFILE!\n");
    ret = access(bmcpu_rambootfile, F_OK|R_OK);
    FAIL_IF_ERROR(ret<0, "please check BMCPU_RAMBOOTFILE(%s)!\n", bmcpu_rambootfile);

    FAIL_IF_ERROR(bmx264_libfile==NULL, "please set env BMX264_LIBFILE!\n");
    ret = access(bmx264_libfile, F_OK|R_OK);
    FAIL_IF_ERROR(ret<0, "please check BMX264_LIBFILE(%s)!\n", bmx264_libfile);

    av_log(avctx, AV_LOG_INFO, "BMCPU_FIPFILE = %s\n", bmcpu_fipfile);
    av_log(avctx, AV_LOG_INFO, "BMCPU_RAMBOOTFILE = %s\n", bmcpu_rambootfile);
    av_log(avctx, AV_LOG_INFO, "BMX264_LIBFILE = %s\n", bmx264_libfile);

    gettimeofday(&ps, NULL);

    ret =  bmcpu_get_cpu_status(bmx->device_handle);
    FAIL_IF_ERROR(ret!=1, "Failed to bmcpu_get_cpu_status!\n");

    // ret = bmcpu_start_cpu(bmx->device_handle, bmcpu_fipfile, bmcpu_rambootfile);
    // FAIL_IF_ERROR(ret!=0, "Failed to bmcpu_start_cpu()!\n");

    gettimeofday(&pe, NULL);
    av_log(avctx, AV_LOG_INFO, "boot OS on A53: %.3fSec\n",
           ((pe.tv_sec-ps.tv_sec) + (pe.tv_usec-ps.tv_usec)/(1000*1000*1.0)));

    /* log level must be set before opening process */
    if (bmx264_logfile)
        ret = bmcpu_set_log(bmx->device_handle, BMCPU_LOGLEVEL_ERROR, LOG_TO_CONSOLE_OFF, -1);
    else
        ret = bmcpu_set_log(bmx->device_handle, BMCPU_LOGLEVEL_NONE, LOG_TO_CONSOLE_OFF, -1);
    FAIL_IF_ERROR(ret!=0, "Failed to set bmcpu log level!\n");

    bmx->proc_handle = bmcpu_open_process(bmx->device_handle, 0, -1);
    FAIL_IF_ERROR(bmx->proc_handle<=0, "Failed to bmcpu_open_process()! handle=%d\n",
                  bmx->proc_handle); // TODO

    if (bmx264_logfile) {
        av_log(avctx, AV_LOG_INFO, "BMX264_LOGFILE = %s\n", bmx264_logfile);
        ret = bmcpu_get_log(bmx->device_handle, bmx->proc_handle, bmx264_logfile, -1);
        FAIL_IF_ERROR(ret!=0, "Failed to get bmcpu log!\n");
    }

    ret = bmcpu_load_library(bmx->device_handle, bmx->proc_handle, bmx264_libfile, -1);
    FAIL_IF_ERROR(ret!=0, "Failed to load %s\n", bmx264_libfile);

    bmx->msg_size[0] = sizeof(bmx264_msg1_t);
    bmx->msg_size[1] = sizeof(bmx264_msg2_t);
    bmx->msg_size[2] = sizeof(bmx264_msg3_t);
    bmx->msg_size[3] = sizeof(bmx264_msg4_t);

    for (int i=0; i<4; i++) {
        /* get device memory for msg1 */
        ret = bm_malloc_device_byte(bmx->device_handle, &bmx->msg_d[i], bmx->msg_size[i]);
        FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

        /* get the va of msg1 device memory */
        bmx->msg_d_va[i] = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                               (void*)(bmx->msg_d[i].u.device.device_addr), bmx->msg_size[i], -1);
        FAIL_IF_ERROR(bmx->msg_d_va[i]==NULL, "Failed to map device physical address\n");
    }

    /* get device memory for x264 parameter */
    ret = bm_malloc_device_byte(bmx->device_handle, &bmx->params_d, params_size);
    FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

    /* get the va of x264 parameter */
    bmx->params_d_va = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                           (void*)(bmx->params_d.u.device.device_addr), params_size, -1);
    FAIL_IF_ERROR(bmx->params_d_va==NULL, "Failed to map device physical address\n");

    /* get device memory for x264 picture */
    ret = bm_malloc_device_byte(bmx->device_handle, &bmx->pic_d, pic_size);
    FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

    /* get the va of x264 picture */
    bmx->pic_d_va = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                        (void*)(bmx->pic_d.u.device.device_addr), pic_size, -1);
    FAIL_IF_ERROR(bmx->pic_d_va==NULL, "Failed to map device physical address\n");

    /* get device memory for x264 picture */
    ret = bm_malloc_device_byte(bmx->device_handle, &bmx->pic_out_d, pic_size);
    FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

    /* get the va of x264 picture */
    bmx->pic_out_d_va = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                            (void*)(bmx->pic_out_d.u.device.device_addr), pic_size, -1);
    FAIL_IF_ERROR(bmx->pic_out_d_va==NULL, "Failed to map device physical address\n");

    bmx->nal_s = calloc(size, 1);
    if (bmx->nal_s==NULL)
        goto failed;

    /* get device memory for x264 nal */
    ret = bm_malloc_device_byte(bmx->device_handle, &bmx->nal_d, size);
    FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

    /* get the va of x264 nal */
    bmx->nal_d_va = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                        (void*)(bmx->nal_d.u.device.device_addr), size, -1);
    FAIL_IF_ERROR(bmx->nal_d_va==NULL, "Failed to map device physical address\n");

    bmx->np_s.p_nnal    = (int*)(bmx->nal_s);
    bmx->np_s.p_nal     = (x264_nal_t*)(bmx->nal_s + sizeof(int));
    bmx->np_s.p_payload = bmx->nal_s + sizeof(int) + MAX_NUM_NAL*sizeof(x264_nal_t);

    return 0;
failed:

    return -1;
}

int bmx264_deinit(AVCodecContext* avctx)
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;
    int ret;

    if (bmx==NULL) {
        goto failed;
    }
    if (bmx->device_handle==NULL) {
        goto failed;
    }
    if (bmx->proc_handle<0) {
        goto failed;
    }

    if (bmx->nal_d_va) {
        bm_free_device(bmx->device_handle, bmx->nal_d);
        bmx->nal_d_va = NULL;
    }

    if (bmx->nal_s) {
        free(bmx->nal_s);
        bmx->nal_s = NULL;
    }

    if (bmx->pic_out_d_va) {
        bm_free_device(bmx->device_handle, bmx->pic_out_d);
        bmx->pic_out_d_va = NULL;
    }

    if (bmx->pic_d_va) {
        bm_free_device(bmx->device_handle, bmx->pic_d);
        bmx->pic_d_va = NULL;
    }

    if (bmx->params_d_va) {
        bm_free_device(bmx->device_handle, bmx->params_d);
        bmx->params_d_va = NULL;
    }

    for (int i=0; i<4; i++) {
        if (bmx->msg_d_va[i]) {
            bm_free_device(bmx->device_handle, bmx->msg_d[i]);
            bmx->msg_d_va[i] = NULL;
        }
    }

    if (bmx->device_handle) {
        if (bmx->proc_handle>0) { // TODO
            ret = bmcpu_close_process(bmx->device_handle, bmx->proc_handle, -1);
            FAIL_IF_ERROR(ret<0, "Failed to bmcpu_close_process()\n");
            bmx->proc_handle = -1; // TODO
        }

        bm_dev_free(bmx->device_handle);
        bmx->device_handle = NULL;
    }

    return 0;
failed:
    return -1;
}

/* pix_fmt: AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_NV12, AV_PIX_FMT_GRAY8 */
int bmx264_pic_alloc(AVCodecContext* avctx, int y_size, int cbcr_size, int pix_fmt)
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;
    int frame_size = y_size;
    int plane_offset[3] = {0, y_size, 0};
    int ret;

    if (pix_fmt==AV_PIX_FMT_YUV420P || pix_fmt==AV_PIX_FMT_YUVJ420P) {
        frame_size += 2*cbcr_size;
        plane_offset[2] = y_size + cbcr_size;
    } else if (pix_fmt==AV_PIX_FMT_NV12) {
        frame_size += cbcr_size;
    }

    /* get device memory for x264 picture */
    ret = bm_malloc_device_byte(bmx->device_handle, &bmx->plane_d, frame_size);
    FAIL_IF_ERROR(ret!=0, "Failed to allocate device memory\n");

    /* get the va of x264 picture */
    bmx->plane_d_va[0] = bmcpu_map_phys_addr(bmx->device_handle, bmx->proc_handle,
                                             (void*)(bmx->plane_d.u.device.device_addr), frame_size, -1);
    FAIL_IF_ERROR(bmx->plane_d_va[0]==NULL, "Failed to map device physical address\n");

    if (pix_fmt==AV_PIX_FMT_YUV420P || pix_fmt==AV_PIX_FMT_YUVJ420P) {
        bmx->plane_d_va[1] = bmx->plane_d_va[0] + plane_offset[1];
        bmx->plane_d_va[2] = bmx->plane_d_va[0] + plane_offset[2];
    } else if (pix_fmt==AV_PIX_FMT_NV12) {
        bmx->plane_d_va[1] = bmx->plane_d_va[0] + plane_offset[1];
        bmx->plane_d_va[2] = NULL;
    } else {
        bmx->plane_d_va[1] = NULL;
        bmx->plane_d_va[2] = NULL;
    }

    return 0;

failed:
    return -1;
}

int bmx264_pic_free(AVCodecContext* avctx)
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;

    if (bmx->plane_d_va[0]) {
        bm_free_device(bmx->device_handle, bmx->plane_d);
        bmx->plane_d_va[0] = NULL;
        bmx->plane_d_va[1] = NULL;
        bmx->plane_d_va[2] = NULL;
    }

    return 0;
}

long bmx264_dispatch_task(AVCodecContext* avctx, const char func_name[], uint8_t* func_param_va)
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;
    bmx264_msg1_t msg1 = {0};
    char wrapper_func_name[128] = "x264_run_task";
    char wrapper_func_par_name[128] = {0};
    int  wrapper_func_par_size;
    int ret;

    strcpy(msg1.header.func, func_name);
    msg1.header.par_num = 1;
    msg1.par[0].type    = 0;
    sprintf(msg1.par[0].name, "%p", func_param_va);
    msg1.par[0].size    = strlen(msg1.par[0].name)+1;

    /* upload data from host to a53 */
    ret = bm_memcpy_s2d(bmx->device_handle, bmx->msg_d[0], &msg1);
    FAIL_IF_ERROR(ret!=0, "Failed to copy data from host to device\n");

    /* exec function on a53 */
    sprintf(wrapper_func_par_name, "%p", bmx->msg_d_va[0]);
    wrapper_func_par_size = strlen(wrapper_func_par_name)+1;

#if 0
    av_log(avctx, AV_LOG_INFO, "\nwrapper func name: %s\n", wrapper_func_name);
    av_log(avctx, AV_LOG_INFO, "wrapper func par name: %s\n", wrapper_func_par_name);
    av_log(avctx, AV_LOG_INFO, "wrapper func par size: %d\n", wrapper_func_par_size);
#endif

    ret = bmcpu_exec_function(bmx->device_handle, bmx->proc_handle,
                              (char *)wrapper_func_name, wrapper_func_par_name, wrapper_func_par_size, -1);
    FAIL_IF_ERROR(ret!=0, "Failed to exec %s on device 0. ret=%d\n", wrapper_func_name, ret);

    memset(&msg1, 0, sizeof(msg1));
    ret = bm_memcpy_d2s(bmx->device_handle, &msg1, bmx->msg_d[0]);
    FAIL_IF_ERROR(ret!=0, "Failed to copy func par from device to host\n");

#if 0
    av_log(avctx, AV_LOG_INFO, "func name: %s\n", msg1.header.func);
    av_log(avctx, AV_LOG_INFO, "return value: %ld\n", msg1.header.ret_val);
    av_log(avctx, AV_LOG_INFO, "par num: %d\n", msg1.header.par_num);
    av_log(avctx, AV_LOG_INFO, "par: type %d, size %d, name %s\n",
           msg1.par[0].type, msg1.par[0].size, msg1.par[0].name);
    av_log(avctx, AV_LOG_INFO, "\n");
#endif

    return msg1.header.ret_val;

failed:
    return -1L;
}

long bmx264_dispatch_task2(AVCodecContext* avctx, bmx264_msg_header_t* header, bmx264_msg_par_t par[])
{
    BMX264Context* bmx = (BMX264Context*)avctx->priv_data;
    char wrapper_func_name[128] = "x264_run_task";
    char wrapper_func_par_name[128] = {0};
    int wrapper_func_par_size = 0;
    int ret;

    if (header->par_num<1 || header->par_num>4) {
        return -1L;
    }

    memset(&(bmx->msgN), 0, sizeof(bmx->msgN));

    /* upload data from host to a53 */
    memcpy(&(bmx->msgN.header), header, sizeof(bmx264_msg_header_t));
    memcpy(&(bmx->msgN.par[0]), &(par[0]), header->par_num*sizeof(bmx264_msg_par_t));

    ret = bm_memcpy_s2d(bmx->device_handle, bmx->msg_d[header->par_num-1], &(bmx->msgN));
    FAIL_IF_ERROR(ret!=0, "Failed to copy data from host to device\n");
    sprintf(wrapper_func_par_name, "%p", bmx->msg_d_va[header->par_num-1]);
    wrapper_func_par_size = strlen(wrapper_func_par_name)+1;

#if 0
    av_log(avctx, AV_LOG_INFO, "\nwrapper func name: %s\n", wrapper_func_name);
    av_log(avctx, AV_LOG_INFO, "wrapper func par name: %s\n", wrapper_func_par_name);
    av_log(avctx, AV_LOG_INFO, "wrapper func par size: %d\n", wrapper_func_par_size);
#endif

    /* exec function on a53 */

    /* 20sec at default. set as 2sec */
    ret = bmcpu_exec_function(bmx->device_handle, bmx->proc_handle,
                              (char *)wrapper_func_name, wrapper_func_par_name, wrapper_func_par_size, 2000);
    FAIL_IF_ERROR(ret!=0, "Failed to exec %s on device 0. ret=%d\n", wrapper_func_name, ret);

    memset(&(bmx->msgN), 0, sizeof(bmx->msgN));
    ret = bm_memcpy_d2s(bmx->device_handle, &(bmx->msgN), bmx->msg_d[header->par_num-1]);
    FAIL_IF_ERROR(ret!=0, "Failed to copy func par from device to host\n");

#if 0
    av_log(avctx, AV_LOG_INFO, "func name: %s\n", bmx->msgN.header.func);
    av_log(avctx, AV_LOG_INFO, "return value: %ld\n", bmx->msgN.header.ret_val);
    av_log(avctx, AV_LOG_INFO, "par num: %d\n", bmx->msgN.header.par_num);
    for (int i=0; i<header->par_num; i++) {
        av_log(avctx, AV_LOG_INFO, "par: type %d, size %d, name %s\n",
               bmx->msgN.par[i].type , bmx->msgN.par[i].size, bmx->msgN.par[i].name);
    }
    av_log(avctx, AV_LOG_INFO, "\n");
#endif

    return bmx->msgN.header.ret_val;

failed:
    return -1L;
}

static int parse_enum(const char *arg, const char *names[], int *dst)
{
    for(int i = 0; names[i]; i++)
        if(!strcasecmp(arg, names[i])) {
            *dst = i;
            return 0;
        }
    return -1;
}

static int parse_cqm(const char *str, uint8_t *cqm, int length)
{
    int i = 0;
    do {
        int coef;
        if(!sscanf(str, "%d", &coef) || coef < 1 || coef > 255)
            return -1;
        cqm[i++] = coef;
    } while(i < length && (str = strchr(str, ',')) && str++);
    return (i == length) ? 0 : -1;
}

static int atobool_internal(const char *str, int *b_error)
{
    if(!strcmp(str, "1") ||
       !strcasecmp(str, "true") ||
       !strcasecmp(str, "yes"))
        return 1;
    if(!strcmp(str, "0") ||
       !strcasecmp(str, "false") ||
       !strcasecmp(str, "no"))
        return 0;
    *b_error = 1;
    return 0;
}

static int atoi_internal(const char *str, int *b_error)
{
    char *end;
    int v = strtol(str, &end, 0);
    if(end == str || *end != '\0')
        *b_error = 1;
    return v;
}

static double atof_internal(const char *str, int *b_error)
{
    char *end;
    double v = strtod(str, &end);
    if(end == str || *end != '\0')
        *b_error = 1;
    return v;
}

#define atobool(str) (name_was_bool = 1, atobool_internal(str, &b_error))
#undef atoi
#undef atof
#define atoi(str) atoi_internal(str, &b_error)
#define atof(str) atof_internal(str, &b_error)

int bmx264_param_parse(x264_param_t *p, const char *name, const char *value)
{
    char *name_buf = NULL;
    int b_error = 0;
    int errortype = -2; // bad value
    int name_was_bool;
    int value_was_null = !value;

    if(!name)
        return -1;
    if(!value)
        value = "true";

    if(value[0] == '=')
        value++;

    if(strchr(name, '_')) // s/_/-/g
    {
        char *c;
        name_buf = strdup(name);
        if(!name_buf)
            return -1;
        while((c = strchr(name_buf, '_')))
            *c = '-';
        name = name_buf;
    }

    if(!strncmp(name, "no", 2))
    {
        name += 2;
        if(name[0] == '-')
            name++;
        value = atobool(value) ? "false" : "true";
    }
    name_was_bool = 0;

#define OPT(STR) else if(!strcmp(name, STR))
#define OPT2(STR0, STR1) else if(!strcmp(name, STR0) || !strcmp(name, STR1))
    if(0);
    OPT("threads") {
        if(!strcasecmp(value, "auto"))
            p->i_threads = X264_THREADS_AUTO;
        else
            p->i_threads = atoi(value);
    } OPT("lookahead-threads") {
        if(!strcasecmp(value, "auto"))
            p->i_lookahead_threads = X264_THREADS_AUTO;
        else
            p->i_lookahead_threads = atoi(value);
    } OPT("sliced-threads")
        p->b_sliced_threads = atobool(value);
    OPT("sync-lookahead") {
        if(!strcasecmp(value, "auto"))
            p->i_sync_lookahead = X264_SYNC_LOOKAHEAD_AUTO;
        else
            p->i_sync_lookahead = atoi(value);
    } OPT2("deterministic", "n-deterministic")
        p->b_deterministic = atobool(value);
    OPT("cpu-independent")
        p->b_cpu_independent = atobool(value); // TODO
    OPT2("level", "level-idc") {
        if(!strcmp(value, "1b"))
            p->i_level_idc = 9;
        else if(atof(value) < 7)
            p->i_level_idc = (int)(10*atof(value)+.5);
        else
            p->i_level_idc = atoi(value);
    } OPT("sar") {
        b_error = (2 != sscanf(value, "%d:%d", &p->vui.i_sar_width, &p->vui.i_sar_height) &&
                    2 != sscanf(value, "%d/%d", &p->vui.i_sar_width, &p->vui.i_sar_height));
    } OPT("overscan")
        b_error |= parse_enum(value, bmx264_overscan_names, &p->vui.i_overscan);
    OPT("videoformat")
        b_error |= parse_enum(value, bmx264_vidformat_names, &p->vui.i_vidformat);
    OPT("fullrange")
        b_error |= parse_enum(value, bmx264_fullrange_names, &p->vui.b_fullrange);
    OPT("colorprim")
        b_error |= parse_enum(value, bmx264_colorprim_names, &p->vui.i_colorprim);
    OPT("transfer")
        b_error |= parse_enum(value, bmx264_transfer_names, &p->vui.i_transfer);
    OPT("colormatrix")
        b_error |= parse_enum(value, bmx264_colmatrix_names, &p->vui.i_colmatrix);
    OPT("chromaloc") {
        p->vui.i_chroma_loc = atoi(value);
        b_error = (p->vui.i_chroma_loc < 0 || p->vui.i_chroma_loc > 5);
    } OPT("alternative-transfer")
        b_error |= parse_enum(value, bmx264_transfer_names, &p->i_alternative_transfer);
    OPT("fps") {
        if(sscanf(value, "%u/%u", &p->i_fps_num, &p->i_fps_den) != 2) {
            double fps = atof(value);
            if(fps > 0.0 && fps <= INT_MAX/1000.0) {
                p->i_fps_num = (int)(fps * 1000.0 + .5);
                p->i_fps_den = 1000;
            } else {
                p->i_fps_num = atoi(value);
                p->i_fps_den = 1;
            }
        }
    } OPT2("ref", "frameref")
        p->i_frame_reference = atoi(value);
    OPT("dpb-size")
        p->i_dpb_size = atoi(value);
    OPT("keyint") {
        if(strstr(value, "infinite"))
            p->i_keyint_max = KEYINT_MAX_INFINITE;
        else
            p->i_keyint_max = atoi(value);
    } OPT2("min-keyint", "keyint-min") {
        p->i_keyint_min = atoi(value);
        if(p->i_keyint_max < p->i_keyint_min)
            p->i_keyint_max = p->i_keyint_min;
    } OPT("scenecut") {
        p->i_scenecut_threshold = atobool(value);
        if(b_error || p->i_scenecut_threshold) {
            b_error = 0;
            p->i_scenecut_threshold = atoi(value);
        }
    } OPT("intra-refresh")
        p->b_intra_refresh = atobool(value);
    OPT("bframes")
        p->i_bframe = atoi(value);
    OPT("b-adapt") {
        p->i_bframe_adaptive = atobool(value);
        if(b_error) {
            b_error = 0;
            p->i_bframe_adaptive = atoi(value);
        }
    } OPT("b-bias")
        p->i_bframe_bias = atoi(value);
    OPT("b-pyramid") {
        b_error |= parse_enum(value, bmx264_b_pyramid_names, &p->i_bframe_pyramid);
        if(b_error) {
            b_error = 0;
            p->i_bframe_pyramid = atoi(value);
        }
    } OPT("open-gop")
        p->b_open_gop = atobool(value);
    OPT("nf")
        p->b_deblocking_filter = !atobool(value);
    OPT2("filter", "deblock") {
        if(2 == sscanf(value, "%d:%d", &p->i_deblocking_filter_alphac0, &p->i_deblocking_filter_beta) ||
            2 == sscanf(value, "%d,%d", &p->i_deblocking_filter_alphac0, &p->i_deblocking_filter_beta)) {
            p->b_deblocking_filter = 1;
        } else if(sscanf(value, "%d", &p->i_deblocking_filter_alphac0)) {
            p->b_deblocking_filter = 1;
            p->i_deblocking_filter_beta = p->i_deblocking_filter_alphac0;
        } else
            p->b_deblocking_filter = atobool(value);
    } OPT("slice-max-size")
        p->i_slice_max_size = atoi(value);
    OPT("slice-max-mbs")
        p->i_slice_max_mbs = atoi(value);
    OPT("slice-min-mbs")
        p->i_slice_min_mbs = atoi(value);
    OPT("slices")
        p->i_slice_count = atoi(value);
    OPT("slices-max")
        p->i_slice_count_max = atoi(value);
    OPT("cabac")
        p->b_cabac = atobool(value);
    OPT("cabac-idc")
        p->i_cabac_init_idc = atoi(value);
    OPT("constrained-intra")
        p->b_constrained_intra = atobool(value);
    OPT("cqm") {
        if(strstr(value, "flat"))
            p->i_cqm_preset = X264_CQM_FLAT;
        else if(strstr(value, "jvt"))
            p->i_cqm_preset = X264_CQM_JVT;
        else
            p->psz_cqm_file = strdup(value);
    } OPT("cqmfile")
        p->psz_cqm_file = strdup(value); // TODO
    OPT("cqm4") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4iy, 16);
        b_error |= parse_cqm(value, p->cqm_4py, 16);
        b_error |= parse_cqm(value, p->cqm_4ic, 16);
        b_error |= parse_cqm(value, p->cqm_4pc, 16);
    } OPT("cqm8") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_8iy, 64);
        b_error |= parse_cqm(value, p->cqm_8py, 64);
        b_error |= parse_cqm(value, p->cqm_8ic, 64);
        b_error |= parse_cqm(value, p->cqm_8pc, 64);
    } OPT("cqm4i") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4iy, 16);
        b_error |= parse_cqm(value, p->cqm_4ic, 16);
    } OPT("cqm4p") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4py, 16);
        b_error |= parse_cqm(value, p->cqm_4pc, 16);
    } OPT("cqm4iy") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4iy, 16);
    } OPT("cqm4ic") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4ic, 16);
    } OPT("cqm4py") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4py, 16);
    } OPT("cqm4pc") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_4pc, 16);
    } OPT("cqm8i") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_8iy, 64);
        b_error |= parse_cqm(value, p->cqm_8ic, 64);
    } OPT("cqm8p") {
        p->i_cqm_preset = X264_CQM_CUSTOM;
        b_error |= parse_cqm(value, p->cqm_8py, 64);
        b_error |= parse_cqm(value, p->cqm_8pc, 64);
    } OPT("log")
        p->i_log_level = atoi(value);
    OPT2("analyse", "partitions") {
        p->analyse.inter = 0;
        if(strstr(value, "none"))  p->analyse.inter =  0;
        if(strstr(value, "all"))   p->analyse.inter = ~0;

        if(strstr(value, "i4x4"))  p->analyse.inter |= X264_ANALYSE_I4x4;
        if(strstr(value, "i8x8"))  p->analyse.inter |= X264_ANALYSE_I8x8;
        if(strstr(value, "p8x8"))  p->analyse.inter |= X264_ANALYSE_PSUB16x16;
        if(strstr(value, "p4x4"))  p->analyse.inter |= X264_ANALYSE_PSUB8x8;
        if(strstr(value, "b8x8"))  p->analyse.inter |= X264_ANALYSE_BSUB16x16;
    } OPT("8x8dct")
        p->analyse.b_transform_8x8 = atobool(value);
    OPT2("weightb", "weight-b")
        p->analyse.b_weighted_bipred = atobool(value);
    OPT("weightp")
        p->analyse.i_weighted_pred = atoi(value);
    OPT2("direct", "direct-pred")
        b_error |= parse_enum(value, bmx264_direct_pred_names, &p->analyse.i_direct_mv_pred);
    OPT("chroma-qp-offset")
        p->analyse.i_chroma_qp_offset = atoi(value);
    OPT("me")
        b_error |= parse_enum(value, bmx264_motion_est_names, &p->analyse.i_me_method);
    OPT2("merange", "me-range")
        p->analyse.i_me_range = atoi(value);
    OPT2("mvrange", "mv-range")
        p->analyse.i_mv_range = atoi(value);
    OPT2("mvrange-thread", "mv-range-thread")
        p->analyse.i_mv_range_thread = atoi(value);
    OPT2("subme", "subq")
        p->analyse.i_subpel_refine = atoi(value);
    OPT("psy-rd") {
        if(2 == sscanf(value, "%f:%f", &p->analyse.f_psy_rd, &p->analyse.f_psy_trellis) ||
            2 == sscanf(value, "%f,%f", &p->analyse.f_psy_rd, &p->analyse.f_psy_trellis) ||
            2 == sscanf(value, "%f|%f", &p->analyse.f_psy_rd, &p->analyse.f_psy_trellis)) {
        } else if(sscanf(value, "%f", &p->analyse.f_psy_rd)) {
            p->analyse.f_psy_trellis = 0;
        } else {
            p->analyse.f_psy_rd = 0;
            p->analyse.f_psy_trellis = 0;
        }
    } OPT("psy")
        p->analyse.b_psy = atobool(value);
    OPT("chroma-me")
        p->analyse.b_chroma_me = atobool(value);
    OPT("mixed-refs")
        p->analyse.b_mixed_references = atobool(value);
    OPT("trellis")
        p->analyse.i_trellis = atoi(value);
    OPT("fast-pskip")
        p->analyse.b_fast_pskip = atobool(value);
    OPT("dct-decimate")
        p->analyse.b_dct_decimate = atobool(value);
    OPT("deadzone-inter")
        p->analyse.i_luma_deadzone[0] = atoi(value);
    OPT("deadzone-intra")
        p->analyse.i_luma_deadzone[1] = atoi(value);
    OPT("nr")
        p->analyse.i_noise_reduction = atoi(value);
    OPT("bitrate") {
        p->rc.i_bitrate = atoi(value);
        p->rc.i_rc_method = X264_RC_ABR;
    } OPT2("qp", "qp_constant") {
        p->rc.i_qp_constant = atoi(value);
        p->rc.i_rc_method = X264_RC_CQP;
    } OPT("crf") {
        p->rc.f_rf_constant = atof(value);
        p->rc.i_rc_method = X264_RC_CRF;
    } OPT("crf-max")
        p->rc.f_rf_constant_max = atof(value);
    OPT("rc-lookahead")
        p->rc.i_lookahead = atoi(value);
    OPT2("qpmin", "qp-min")
        p->rc.i_qp_min = atoi(value);
    OPT2("qpmax", "qp-max")
        p->rc.i_qp_max = atoi(value);
    OPT2("qpstep", "qp-step")
        p->rc.i_qp_step = atoi(value);
    OPT("ratetol")
        p->rc.f_rate_tolerance = !strncmp("inf", value, 3) ? 1e9 : atof(value);
    OPT("vbv-maxrate")
        p->rc.i_vbv_max_bitrate = atoi(value);
    OPT("vbv-bufsize")
        p->rc.i_vbv_buffer_size = atoi(value);
    OPT("vbv-init")
        p->rc.f_vbv_buffer_init = atof(value);
    OPT2("ipratio", "ip-factor")
        p->rc.f_ip_factor = atof(value);
    OPT2("pbratio", "pb-factor")
        p->rc.f_pb_factor = atof(value);
    OPT("aq-mode")
        p->rc.i_aq_mode = atoi(value);
    OPT("aq-strength")
        p->rc.f_aq_strength = atof(value);
    OPT("qcomp")
        p->rc.f_qcompress = atof(value);
    OPT("mbtree")
        p->rc.b_mb_tree = atobool(value);
    OPT("qblur")
        p->rc.f_qblur = atof(value);
    OPT2("cplxblur", "cplx-blur")
        p->rc.f_complexity_blur = atof(value);
    OPT("zones")
        p->rc.psz_zones = strdup(value);
    OPT("crop-rect")
        b_error |= sscanf(value, "%d,%d,%d,%d", &p->crop_rect.i_left, &p->crop_rect.i_top,
                                                 &p->crop_rect.i_right, &p->crop_rect.i_bottom) != 4;
    OPT("psnr")
        p->analyse.b_psnr = atobool(value);
    OPT("ssim")
        p->analyse.b_ssim = atobool(value);
    OPT("aud")
        p->b_aud = atobool(value);
    OPT("sps-id")
        p->i_sps_id = atoi(value);
    OPT("global-header")
        p->b_repeat_headers = !atobool(value);
    OPT("repeat-headers")
        p->b_repeat_headers = atobool(value);
    OPT("annexb")
        p->b_annexb = atobool(value);
    OPT("force-cfr")
        p->b_vfr_input = !atobool(value);
    OPT("nal-hrd")
        b_error |= parse_enum(value, bmx264_nal_hrd_names, &p->i_nal_hrd);
    OPT("filler")
        p->rc.b_filler = atobool(value);
    OPT("pic-struct")
        p->b_pic_struct = atobool(value);
    OPT("frame-packing")
        p->i_frame_packing = atoi(value);
    OPT("stitchable")
        p->b_stitchable = atobool(value);
    else {
        b_error = 1;
        errortype = -1;
    }
#undef OPT
#undef OPT2
#undef atobool
#undef atoi
#undef atof

    if(name_buf)
        free(name_buf);

    b_error |= value_was_null && !name_was_bool;
    return b_error ? errortype : 0;
}

char* bmx264_param2string(x264_param_t *p, int b_res)
{
    int len = 1000;
    char *buf, *s;
    if(p->rc.psz_zones)
        len += strlen(p->rc.psz_zones);
    buf = s = malloc(len);
    if(!buf)
        return NULL;

    memset(s, 0, len);

    if(b_res) {
        s += sprintf(s, "%dx%d ", p->i_width, p->i_height);
        s += sprintf(s, "fps=%u/%u ", p->i_fps_num, p->i_fps_den);
        s += sprintf(s, "timebase=%u/%u ", p->i_timebase_num, p->i_timebase_den);
        s += sprintf(s, "bitdepth=%d ", p->i_bitdepth);
    }

    if(p->b_opencl)
        s += sprintf(s, "opencl=%d ", p->b_opencl); // TODO
    s += sprintf(s, "cabac=%d", p->b_cabac);
    s += sprintf(s, " ref=%d", p->i_frame_reference);
    s += sprintf(s, " deblock=%d:%d:%d", p->b_deblocking_filter,
                  p->i_deblocking_filter_alphac0, p->i_deblocking_filter_beta);
    s += sprintf(s, " analyse=%#x:%#x", p->analyse.intra, p->analyse.inter);
    s += sprintf(s, " me=%s", bmx264_motion_est_names[ p->analyse.i_me_method ]);
    s += sprintf(s, " subme=%d", p->analyse.i_subpel_refine);
    s += sprintf(s, " psy=%d", p->analyse.b_psy);
    if(p->analyse.b_psy)
        s += sprintf(s, " psy_rd=%.2f:%.2f", p->analyse.f_psy_rd, p->analyse.f_psy_trellis);
    s += sprintf(s, " mixed_ref=%d", p->analyse.b_mixed_references);
    s += sprintf(s, " me_range=%d", p->analyse.i_me_range);
    s += sprintf(s, " chroma_me=%d", p->analyse.b_chroma_me);
    s += sprintf(s, " trellis=%d", p->analyse.i_trellis);
    s += sprintf(s, " 8x8dct=%d", p->analyse.b_transform_8x8);
    s += sprintf(s, " cqm=%d", p->i_cqm_preset);
    s += sprintf(s, " deadzone=%d,%d", p->analyse.i_luma_deadzone[0], p->analyse.i_luma_deadzone[1]);
    s += sprintf(s, " fast_pskip=%d", p->analyse.b_fast_pskip);
    s += sprintf(s, " chroma_qp_offset=%d", p->analyse.i_chroma_qp_offset);
    s += sprintf(s, " threads=%d", p->i_threads);
    s += sprintf(s, " lookahead_threads=%d", p->i_lookahead_threads);
    s += sprintf(s, " sliced_threads=%d", p->b_sliced_threads);
    if(p->i_slice_count)
        s += sprintf(s, " slices=%d", p->i_slice_count);
    if(p->i_slice_count_max)
        s += sprintf(s, " slices_max=%d", p->i_slice_count_max);
    if(p->i_slice_max_size)
        s += sprintf(s, " slice_max_size=%d", p->i_slice_max_size);
    if(p->i_slice_max_mbs)
        s += sprintf(s, " slice_max_mbs=%d", p->i_slice_max_mbs);
    if(p->i_slice_min_mbs)
        s += sprintf(s, " slice_min_mbs=%d", p->i_slice_min_mbs);
    s += sprintf(s, " nr=%d", p->analyse.i_noise_reduction);
    s += sprintf(s, " decimate=%d", p->analyse.b_dct_decimate);
    s += sprintf(s, " interlaced=%s", p->b_interlaced ? p->b_tff ? "tff" : "bff" : p->b_fake_interlaced ? "fake" : "0"); // TODO
    s += sprintf(s, " bluray_compat=%d", p->b_bluray_compat); // TODO
    if(p->b_stitchable)
        s += sprintf(s, " stitchable=%d", p->b_stitchable);

    s += sprintf(s, " constrained_intra=%d", p->b_constrained_intra);

    s += sprintf(s, " bframes=%d", p->i_bframe);
    if(p->i_bframe) {
        s += sprintf(s, " b_pyramid=%d b_adapt=%d b_bias=%d direct=%d weightb=%d open_gop=%d",
                      p->i_bframe_pyramid, p->i_bframe_adaptive, p->i_bframe_bias,
                      p->analyse.i_direct_mv_pred, p->analyse.b_weighted_bipred, p->b_open_gop);
    }
    s += sprintf(s, " weightp=%d", p->analyse.i_weighted_pred > 0 ? p->analyse.i_weighted_pred : 0);

    if(p->i_keyint_max == KEYINT_MAX_INFINITE)
        s += sprintf(s, " keyint=infinite");
    else
        s += sprintf(s, " keyint=%d", p->i_keyint_max);
    s += sprintf(s, " keyint_min=%d scenecut=%d intra_refresh=%d",
                  p->i_keyint_min, p->i_scenecut_threshold, p->b_intra_refresh);

    if(p->rc.b_mb_tree || p->rc.i_vbv_buffer_size)
        s += sprintf(s, " rc_lookahead=%d", p->rc.i_lookahead);

    s += sprintf(s, " rc=%s mbtree=%d", p->rc.i_rc_method == X264_RC_ABR ?
                  (p->rc.b_stat_read ? "2pass" : p->rc.i_vbv_max_bitrate == p->rc.i_bitrate ? "cbr" : "abr")
                  : p->rc.i_rc_method == X264_RC_CRF ? "crf" : "cqp", p->rc.b_mb_tree);
    if(p->rc.i_rc_method == X264_RC_ABR || p->rc.i_rc_method == X264_RC_CRF) {
        if(p->rc.i_rc_method == X264_RC_CRF)
            s += sprintf(s, " crf=%.1f", p->rc.f_rf_constant);
        else
            s += sprintf(s, " bitrate=%d ratetol=%.1f",
                          p->rc.i_bitrate, p->rc.f_rate_tolerance);
        s += sprintf(s, " qcomp=%.2f qpmin=%d qpmax=%d qpstep=%d",
                      p->rc.f_qcompress, p->rc.i_qp_min, p->rc.i_qp_max, p->rc.i_qp_step);
        if(p->rc.b_stat_read)
            s += sprintf(s, " cplxblur=%.1f qblur=%.1f",
                          p->rc.f_complexity_blur, p->rc.f_qblur);
        if(p->rc.i_vbv_buffer_size) {
            s += sprintf(s, " vbv_maxrate=%d vbv_bufsize=%d",
                          p->rc.i_vbv_max_bitrate, p->rc.i_vbv_buffer_size);
            if(p->rc.i_rc_method == X264_RC_CRF)
                s += sprintf(s, " crf_max=%.1f", p->rc.f_rf_constant_max);
        }
    } else if(p->rc.i_rc_method == X264_RC_CQP)
        s += sprintf(s, " qp=%d", p->rc.i_qp_constant);

    if(p->rc.i_vbv_buffer_size)
        s += sprintf(s, " nal_hrd=%s filler=%d", bmx264_nal_hrd_names[p->i_nal_hrd], p->rc.b_filler);
    if(p->crop_rect.i_left | p->crop_rect.i_top | p->crop_rect.i_right | p->crop_rect.i_bottom)
        s += sprintf(s, " crop_rect=%d,%d,%d,%d", p->crop_rect.i_left, p->crop_rect.i_top,
                      p->crop_rect.i_right, p->crop_rect.i_bottom);
    if(p->i_frame_packing >= 0)
        s += sprintf(s, " frame-packing=%d", p->i_frame_packing);

    if(!(p->rc.i_rc_method == X264_RC_CQP && p->rc.i_qp_constant == 0)) {
        s += sprintf(s, " ip_ratio=%.2f", p->rc.f_ip_factor);
        if(p->i_bframe && !p->rc.b_mb_tree)
            s += sprintf(s, " pb_ratio=%.2f", p->rc.f_pb_factor);
        s += sprintf(s, " aq=%d", p->rc.i_aq_mode);
        if(p->rc.i_aq_mode)
            s += sprintf(s, ":%.2f", p->rc.f_aq_strength);
        if(p->rc.psz_zones)
            s += sprintf(s, " zones=%s", p->rc.psz_zones);
        else if(p->rc.i_zones)
            s += sprintf(s, " zones");
    }

    return buf;
}
#endif

