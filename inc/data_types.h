
/**
 * @file data_types.h
 * @author Marcos Dominguez
 * @brief data types definitions.
 *
 *
 * @version 1.0
 * @date 2021-08-05
 *
 *
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef uint8_t bool_t;
#ifndef FALSE
#define FALSE   0
#endif
#ifndef TRUE
#define TRUE    0xFF
#endif

#define MAX_INT16                   0x7FFF
#define MIN_INT16                   0x8000

#define MAX_INT32                   0x7FFFFFFF
#define MIN_INT32                   0x80000000

/**
 * @brief Format of callback method
 *
 */
typedef void (*UtilsCallback_t)(void *);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
