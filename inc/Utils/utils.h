/**
 * @file utils.h
 * @author Marcos Dominguez
 * @brief Tools for create tested code.
 * @version 0.1
 * @date 2024-02-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#ifdef TEST

#define STATIC

#define TESTED_VARIABLE extern

#ifndef   __STATIC_FORCEINLINE
    #define __STATIC_FORCEINLINE
#endif
#define PERCENTAGE(percent, x) ((percent * x) / 100UL)

#define _ALWAYS_INL

#else

#define STATIC static

#define TESTED_VARIABLE

#ifndef   __STATIC_FORCEINLINE
    #define __STATIC_FORCEINLINE       __attribute__((always_inline)) static __inline__
#endif

#define _ALWAYS_INL inline __attribute__((always_inline))

#define PERCENTAGE(percent, x) (MUL_UU(percent, x) / 100)

#endif

#define Q15_SCALE(x)  (int32_t)((x) * (1 << 15))

#ifndef NULL
    #define NULL ((void *)0)
#endif
#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
