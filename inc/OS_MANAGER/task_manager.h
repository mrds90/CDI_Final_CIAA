
/**
 * @file memory_pool.h
 * @author Marcos Dominguez
 * @brief hold memory of OS.
 *
 *
 * @version 1.0
 * @date 2023-08-24
 *
 *
 */

#ifndef MEMORY_POOL_H
#define MEMORY_POOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "osal_task.h"


/*================ PUBLIC MACRO AND CONSTANTS ==========================================*/

#define STACK_SIZE(x)           ((x) * OSAL_ATOMIC_TASK_SIZE)

#define STACK_SIZE_REAL_WORLD       STACK_SIZE(2)
#define STACK_SIZE_CONTROLLER       STACK_SIZE(2)
#define STACK_SIZE_IDENTIFICACION   STACK_SIZE(5)

/*================ PUBLIC DATA TYPE ====================================================*/

/*================ PUBLIC FUNCTION DECLARATIONS ========================================*/

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
