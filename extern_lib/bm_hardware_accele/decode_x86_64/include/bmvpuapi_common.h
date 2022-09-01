/* bmvpuapi API library for the BitMain Sophon SoC
 *
 * Copyright (C) 2018 Solan Shang
 * Copyright (C) 2015 Carlos Rafael Giani
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 * USA
 */

/* This library provides a high-level interface for controlling the BitMain
 * Sophon VPU en/decoder.
 */


#ifndef __BMVPUAPI_COMMON_H__
#define __BMVPUAPI_COMMON_H__

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32) || defined(WIN32) || defined(__WIN32__)
#define ATTRIBUTE 
#define DECL_EXPORT __declspec(dllexport)
#define DECL_IMPORT __declspec(dllimport)
#else
#define ATTRIBUTE __attribute__((deprecated))
#define DECL_EXPORT
#define DECL_IMPORT
#endif
#ifdef __cplusplus
extern "C" {
#endif


/**************************************************/
/******* ALLOCATOR STRUCTURES AND FUNCTIONS *******/
/**************************************************/

/* Typedef for physical addresses */
#ifdef __linux__
typedef unsigned long bmvpu_phys_addr_t;
#elif _WIN32
typedef unsigned long long bmvpu_phys_addr_t;
#endif 

/* BmVpuAllocationFlags: flags for the BmVpuDMABufferAllocator's allocate vfunc */
typedef enum
{
    BM_VPU_ALLOCATION_FLAG_CACHED       = 0,
    BM_VPU_ALLOCATION_FLAG_WRITECOMBINE = 1,
    BM_VPU_ALLOCATION_FLAG_UNCACHED     = 2
} BmVpuAllocationFlags;

#define BM_VPU_ALLOCATION_FLAG_DEFAULT    BM_VPU_ALLOCATION_FLAG_WRITECOMBINE


/* BmVpuMappingFlags: flags for the BmVpuDMABufferAllocator's map function pointers
 * These flags can be bitwise-OR combined, although READ and WRITE cannot
 * both be set */
typedef enum
{
    /* Map memory for CPU write access */
    BM_VPU_MAPPING_FLAG_WRITE   = (1UL << 0),
    /* Map memory for CPU read access */
    BM_VPU_MAPPING_FLAG_READ    = (1UL << 1)
} BmVpuMappingFlags;


typedef struct _BmVpuDMABuffer BmVpuDMABuffer;
typedef struct _BmVpuWrappedDMABuffer BmVpuWrappedDMABuffer;
DECL_EXPORT typedef struct _BmVpuDMABufferAllocator BmVpuDMABufferAllocator;

/* BmVpuDMABufferAllocator:
 *
 * This structure contains function pointers which define an allocator for DMA buffers
 * (= physically contiguous memory blocks). It is possible to define a custom allocator,
 * which is useful for tracing memory allocations, and for hooking up any existing allocation mechanisms,
 * such as ION or CMA.
 *
 * Older allocators like the VPU ones unfortunately work with physical addresses directly, and do not support
 * DMA-BUF or the like. To keep compatible with these older allocators and allowing for newer and better
 * methods, both physical addresses and FDs are supported by this API. Typically, an allocator allows for
 * one of them. If an allocator does not support FDs, get_fd() must return -1. If it does not support physical
 * addresses, then the physical address returned by get_physical_address() must be 0.
 *
 * The pointers are:
 *
 * allocate():
 *   Allocates a DMA buffer. "size" is the size of the buffer in bytes. "alignment" is the address
 *   alignment in bytes. An alignment of 1 or 0 means that no alignment is required.
 *   "flags" is a bitwise OR combination of flags (or 0 if no flags are used, in which case
 *   cached pages are used by default). See BmVpuAllocationFlags for a list of valid flags.
 *   If allocation fails, NULL is returned.
 *
 * deallocate():
 *   Deallocates a DMA buffer. The buffer must have been allocated with the same allocator.
 *
 * map():
 *   Maps a DMA buffer to the local address space, and returns the virtual address to this space.
 *   "flags" is a bitwise OR combination of flags (or 0 if no flags are used, in which case it will map
 *   in regular read/write mode). See BmVpuMappingFlags for a list of valid flags.
 *
 * unmap():
 *   Unmaps a DMA buffer. If the buffer isn't currently mapped, this function does nothing.
 *
 * get_fd():
 *   Gets the file descriptor associated with the DMA buffer. This is the preferred way of interacting
 *   with DMA buffers. If the underlying allocator does not support FDs, this function returns -1.
 *
 * get_physical_address():
 *   Gets the physical address associated with the DMA buffer. This address points to the
 *   start of the buffer in the physical address space. If no physical addresses are
 *   supported by the allocator, this function returns 0.
 *
 * get_size():
 *   Returns the size of the buffer, in bytes.
 *
 * The pointers get_fd(), get_physical_address(), and get_size() can also be used while the buffer is mapped. */
struct _BmVpuDMABufferAllocator
{
    BmVpuDMABuffer* (*allocate)(BmVpuDMABufferAllocator *allocator, int soc_idx, size_t size, unsigned int alignment, unsigned int flags);
    void (*deallocate)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);

    uint8_t* (*map)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer, unsigned int flags);
    void (*unmap)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);

    int (*get_fd)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);
    bmvpu_phys_addr_t (*get_physical_address)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);

    size_t (*get_size)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);

    int (*flush)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);
    int (*invalidate)(BmVpuDMABufferAllocator *allocator, BmVpuDMABuffer *buffer);
};


/* BmVpuDMABuffer:
 *
 * Opaque object containing a DMA buffer. Its structure is defined by the allocator which created the object. */
struct _BmVpuDMABuffer
{
    BmVpuDMABufferAllocator *allocator;
};


/* BmVpuWrappedDMABuffer:
 *
 * Structure for wrapping existing DMA buffers.
 * This is useful for interfacing with existing buffers that were not allocated by bmvpuapi.
 *
 * fd, physical_address, and size are filled with user-defined values. If the DMA buffer is referred to
 * by a file descriptor, then fd must be set to the descriptor value, otherwise fd must be set to -1.
 * If the buffer is referred to by a physical address, then physical_address must be set to that address,
 * otherwise physical_address must be 0.
 * map_func and unmap_func are used in the bmvpu_dma_buffer_map() / bmvpu_dma_buffer_unmap() calls.
 * If these function pointers are NULL, no mapping will be done. NOTE: bmvpu_dma_buffer_map() will return
 * a NULL pointer in this case.
 */
struct _BmVpuWrappedDMABuffer
{
    BmVpuDMABuffer parent;

    uint8_t* (*map)(BmVpuWrappedDMABuffer *wrapped_dma_buffer, unsigned int flags);
    void (*unmap)(BmVpuWrappedDMABuffer *wrapped_dma_buffer);

    int fd;
    bmvpu_phys_addr_t physical_address;
    size_t size;
};


/* Convenience functions which call the corresponding function pointers in the allocator */
DECL_EXPORT BmVpuDMABuffer* bmvpu_dma_buffer_allocate(BmVpuDMABufferAllocator *allocator, int soc_idx, size_t size,
                                          unsigned int alignment, unsigned int flags);
DECL_EXPORT void bmvpu_dma_buffer_deallocate(BmVpuDMABuffer *buffer);
DECL_EXPORT uint8_t* bmvpu_dma_buffer_map(BmVpuDMABuffer *buffer, unsigned int flags);
DECL_EXPORT void bmvpu_dma_buffer_unmap(BmVpuDMABuffer *buffer);
int bmvpu_dma_buffer_get_fd(BmVpuDMABuffer *buffer);
DECL_EXPORT bmvpu_phys_addr_t bmvpu_dma_buffer_get_physical_address(BmVpuDMABuffer *buffer);
size_t bmvpu_dma_buffer_get_size(BmVpuDMABuffer *buffer);
DECL_EXPORT int bmvpu_dma_buffer_flush(BmVpuDMABuffer *buffer);
int bmvpu_dma_buffer_invalidate(BmVpuDMABuffer *buffer);

/* Convenience predefined allocator for allocating DMA buffers. */
DECL_EXPORT BmVpuDMABufferAllocator* bmvpu_get_default_allocator(void);

/* Call for initializing wrapped DMA buffer structures.
 * Always call this before further using such a structure. */
DECL_EXPORT void bmvpu_init_wrapped_dma_buffer(BmVpuWrappedDMABuffer *buffer);


/* Heap allocation function for virtual memory blocks internally allocated by bmvpuapi.
 * These have nothing to do with the DMA buffer allocation interface defined above.
 * By default, malloc/free are used. */
typedef void* (*BmVpuHeapAllocFunc)(size_t const size, void *context,
                                    char const *file, int const line, char const *fn);
typedef void  (*BmVpuHeapFreeFunc)(void *memblock, size_t const size, void *context,
                                   char const *file, int const line, char const *fn);
/* This function allows for setting custom heap allocators, which are used to create internal heap blocks.
 * The heap allocator referred to by "heap_alloc_fn" must return NULL if allocation fails.
 * "context" is a user-defined value, which is passed on unchanged to the allocator functions.
 * Calling this function with either "heap_alloc_fn" or "heap_free_fn" set to NULL resets the internal
 * pointers to use malloc and free (the default allocators). */
void bmvpu_set_heap_allocator_functions(BmVpuHeapAllocFunc heap_alloc_fn,
                                        BmVpuHeapFreeFunc heap_free_fn,
                                        void *context);




/***********************/
/******* LOGGING *******/
/***********************/


/* Log levels. */
typedef enum
{
    BM_VPU_LOG_LEVEL_ERROR   = 0,
    BM_VPU_LOG_LEVEL_WARNING = 1,
    BM_VPU_LOG_LEVEL_INFO    = 2,
    BM_VPU_LOG_LEVEL_DEBUG   = 3, /* only useful for developers */
    BM_VPU_LOG_LEVEL_LOG     = 4, /* only useful for developers */
    BM_VPU_LOG_LEVEL_TRACE   = 5  /* only useful for developers */
} BmVpuLogLevel;

/* Function pointer type for logging functions.
 *
 * This function is invoked by BM_VPU_LOG() macro calls. This macro also passes the name
 * of the source file, the line in that file, and the function name where the logging occurs
 * to the logging function (over the file, line, and fn arguments, respectively).
 * Together with the log level, custom logging functions can output this metadata, or use
 * it for log filtering etc.*/
typedef void (*BmVpuLoggingFunc)(BmVpuLogLevel level, char const *file, int const line,
                                 char const *fn, const char *format, ...);

/* Defines the threshold for logging. Logs with lower priority are discarded.
 * By default, the threshold is set to BM_VPU_LOG_LEVEL_INFO. */
DECL_EXPORT void bmvpu_set_logging_threshold(BmVpuLogLevel threshold);

/* Defines a custom logging function.
 * If logging_fn is NULL, logging is disabled. This is the default value. */
DECL_EXPORT void bmvpu_set_logging_function(BmVpuLoggingFunc logging_fn);

/* Get the threshold for logging. */
DECL_EXPORT BmVpuLogLevel bmvpu_get_logging_threshold(void);



/******************************************************/
/******* MISCELLANEOUS STRUCTURES AND FUNCTIONS *******/
/******************************************************/


/* Frame types understood by the VPU. */
typedef enum
{
    BM_VPU_FRAME_TYPE_UNKNOWN = 0,
    BM_VPU_FRAME_TYPE_I,
    BM_VPU_FRAME_TYPE_P,
    BM_VPU_FRAME_TYPE_B,
    BM_VPU_FRAME_TYPE_IDR
} BmVpuFrameType;


/* Codec format to use for en/decoding. */
typedef enum
{
    /* H.264.
     * Encoding: Baseline/Constrained Baseline/Main/High/High 10 Profiles Level @ L5.2
     */
    BM_VPU_CODEC_FORMAT_H264,

    /* H.265.
     * Encoding: Supports Main/Main 10/Main Still Picture Profiles
     *           @ L5.1 High-tier
     */
    BM_VPU_CODEC_FORMAT_H265,
} BmVpuCodecFormat;


typedef enum
{
    /* planar 4:2:0; if the chroma_interleave parameter is 1, the corresponding format is NV12, otherwise it is I420 */
    BM_VPU_COLOR_FORMAT_YUV420 = 0,
    /* planar 4:2:2; if the chroma_interleave parameter is 1, the corresponding format is NV16 */
    BM_VPU_COLOR_FORMAT_YUV422 = 1,
    /* planar 4:4:4; if the chroma_interleave parameter is 1, the corresponding format is NV24 */
    BM_VPU_COLOR_FORMAT_YUV444 = 3,
    /* 8-bit greayscale */
    BM_VPU_COLOR_FORMAT_YUV400 = 4
} BmVpuColorFormat;


/* Framebuffers are frame containers, and are used both for en- and decoding. */
typedef struct
{
    /* DMA buffer which contains the pixels. */
    BmVpuDMABuffer *dma_buffer;

    /* Make sure each framebuffer has an ID that is different
     * to the IDs of each other */
    int          myIndex;

    /* Stride of the Y and of the Cb&Cr components.
     * Specified in bytes. */
    unsigned int y_stride;
    unsigned int cbcr_stride; // TODO

    unsigned int width;  /* width of frame buffer */
    unsigned int height; /* height of frame buffer */

    /* These define the starting offsets of each component
     * relative to the start of the buffer. Specified in bytes. */
    size_t y_offset;
    size_t cb_offset;
    size_t cr_offset;

    /* Set to 1 if the framebuffer was already marked as used in encoder.
     * This is for internal use only.
     * Not to be read or written from the outside. */
    int already_marked;

    /* Internal, implementation-defined data. Do not modify. */
    void *internal;

    /* User-defined pointer.
     * The library does not touch this value.
     * This can be used for example to identify which framebuffer out of
     * the initially allocated pool was used by the VPU to contain a frame.
     */
    void *context;
} BmVpuFramebuffer;


/* Structure containing details about encoded frames. */
typedef struct
{
    /* When decoding, data must point to the memory block which contains
     * encoded frame data that gets consumed by the VPU.
     * Only used by the decoder. */
    uint8_t *data;

    /* Size of the encoded data, in bytes. When decoding, this is set by
     * the user, and is the size of the encoded data that is pointed to
     * by data. When encoding, the encoder sets this to the size of the
     * acquired output block, in bytes (exactly the same value as the
     * acquire_output_buffer's size argument). */
    size_t data_size;

    /* Frame type (I, P, B, ..) of the encoded frame. Filled by the encoder.
     * Only used by the encoder. */
    BmVpuFrameType frame_type;

    /* Handle produced by the user-defined acquire_output_buffer function
     * during encoding.
     * Only used by the encoder. */
    void *acquired_handle;

    /* User-defined pointer.
     * The library does not touch this value.
     * This pointer and the one from the corresponding raw frame will have
     * the same value. The library will pass then through. */
    void *context;

    /* User-defined timestamps.
     * In many cases, the context one wants to associate with raw/encoded frames
     * is a PTS-DTS pair. Just like the context pointer, the library just passes
     * them through to the associated raw frame, and does not actually touch
     * their values. */
    uint64_t pts;
    uint64_t dts;

    int src_idx;

    int avg_ctu_qp;
} BmVpuEncodedFrame;


/* Structure containing details about raw, uncompressed frames. */
typedef struct
{
    BmVpuFramebuffer *framebuffer;

    /* User-defined pointer.
     * The library does not touch this value.
     * This pointer and the one from the corresponding encoded frame will have
     * the same value. The library will pass then through. */
    void *context;

    /* User-defined timestamps.
     * In many cases, the context one wants to associate with raw/encoded frames
     * is a PTS-DTS pair. Just like the context pointer, the library just passes
     * them through to the associated encoded frame, and does not actually touch
     * their values. */
    uint64_t pts;
    uint64_t dts;
} BmVpuRawFrame;

/* Structure used together with bmvpu_calc_framebuffer_sizes() */
typedef struct {
    /* Frame width and height, aligned to the 16-pixel boundary required by the VPU. */
    int width;
    int height;

    /* Stride sizes, in bytes, with alignment applied.
     * The Cb and Cr planes always use the same stride. */
    int y_stride; /* aligned stride */
    int c_stride; /* aligned stride (optional) */

    /* Required DMA memory size for the Y,Cb,Cr planes, in bytes.
     * The Cb and Cr planes always are of the same size. */
    int y_size;
    int c_size;

    /* Total required size of a framebuffer's DMA buffer, in bytes.
     * This value includes the sizes of all planes. */
    int size;
} BmVpuFbInfo;

/**
 * Calculate various sizes out of the given width & height and color format.
 * The results are stored in "fb_info".
 * The given frame width and height will be aligned if they aren't already,
 * and the aligned value will be stored in fb_info.
 * Width & height must be nonzero.
 * The fb_info pointer must also be non-NULL.
 * framebuffer_alignment is an alignment value for the sizes of the Y/U/V planes.
 * 0 or 1 mean no alignment.
 * chroma_interleave is set to 1 if a shared CbCr chroma plane is to be used,
 * 0 if Cb and Cr shall use separate planes.
 */
DECL_EXPORT int bmvpu_calc_framebuffer_sizes(int mapType, BmVpuColorFormat color_format,
                                  int frame_width, int frame_height,
                                  int chroma_interleave, BmVpuFbInfo *fb_info);

/**
 * Fill fields of the BmVpuFramebuffer structure, based on data from "fb_info".
 * The specified DMA buffer and context pointer are also set.
 */
DECL_EXPORT int bmvpu_fill_framebuffer_params(BmVpuFramebuffer *framebuffer,
                                   BmVpuFbInfo *fb_info,
                                   BmVpuDMABuffer *fb_dma_buffer,
                                   int fb_id, void* context);

/**
 * Upload data from HOST to a VPU core.
 * For now, only support PCIE mode.
 *
 * return value:
 *   -1, failed
 *    0, done
 */
DECL_EXPORT int bmvpu_upload_data(int vpu_core_idx,
                      const uint8_t* host_va, int host_stride,
                      uint64_t vpu_pa, int vpu_stride,
                      int width, int height);

/**
 * Download data from a VPU core to HOST.
 * For now, only support PCIE mode.
 *
 * return value:
 *   -1, failed
 *    0, done
 */
int bmvpu_download_data(int vpu_core_idx,
                        uint8_t* host_va, int host_stride,
                        uint64_t vpu_pa, int vpu_stride,
                        int width, int height);

/**
 * Returns a human-readable description of the given color format.
 * Useful for logging.
 */
char const *bmvpu_color_format_string(BmVpuColorFormat color_format);

/**
 * Returns a human-readable description of the given frame type.
 * Useful for logging.
 */
char const *bmvpu_frame_type_string(BmVpuFrameType frame_type);


#ifdef __cplusplus
}
#endif

#endif
