#ifndef _MD5_H_
#define _MD5_H_

#include <stdint.h>

#define MD5_DIGEST_LENGTH 16
#define MD5_STRING_LENGTH (MD5_DIGEST_LENGTH * 2)

/**
* @brief    This type is an 8-bit unsigned integral type, which is used for declaring pixel data.
*/
typedef uint8_t         Uint8;

/**
* @brief    This type is a 32-bit unsigned integral type, which is used for declaring variables with wide ranges and no signs such as size of buffer.
*/
typedef uint32_t        Uint32;

/**
* @brief    This type is a 16-bit unsigned integral type.
*/
typedef uint16_t        Uint16;

/**
* @brief    This type is an 8-bit signed integral type.
*/
typedef int8_t          Int8;

/**
* @brief    This type is a 32-bit signed integral type.
*/
typedef int32_t         Int32;

/**
* @brief    This type is a 16-bit signed integral type.
*/

typedef int16_t         Int16;
#if defined(_MSC_VER)
typedef unsigned __int64 Uint64;
typedef __int64          Int64;
#else
typedef uint64_t        Uint64;
typedef int64_t         Int64;
#endif

typedef struct MD5state_st {
    Uint32 A,B,C,D;
    Uint32 Nl,Nh;
    Uint32 data[16];
    Uint32 num;
} MD5_CTX;

Int32 MD5_Init(
    MD5_CTX *c
    );

Int32 MD5_Update(
    MD5_CTX*    c,
    const void* data,
    size_t      len);

Int32 MD5_Final(
    Uint8*      md,
    MD5_CTX*    c
    );

Uint8* MD5(
    const Uint8*  d,
    size_t        n,
    Uint8*        md
    );

void plane_md5(MD5_CTX *md5_ctx,
    Uint8  *src,
    int    src_x,
    int    src_y,
    int    out_x,
    int    out_y,
    int    stride,
    int    bpp,
    Uint16 zero
);

int md5_cmp(unsigned char* got, unsigned char* exp ,int size);
int md5_get(unsigned char* got, int size, char* md5_str, int flag);

#endif /* _MD5_H_ */