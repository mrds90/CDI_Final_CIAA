/**
 * @file osal_global.h
 * @author Marcos Dominguez
 *
 * @brief OS abstraction layer.
 *
 * @version 0.1
 * @date 2023-12-15
 */

#ifndef _OSAL_GLOBAL_H
#define _OSAL_GLOBAL_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#if (OS_USED == OS_FREERTOS)
#include "port_freertos.h"
#elif (OS_USED == OS_CUSTOM)
#include "port_custom.h"
#endif

/// \cond
#include "data_types.h"
/// \endcond

typedef port_tick_t osal_tick_t;


/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

#ifdef  __cplusplus
}

#endif

#endif  /* _INC_TEMPLATE_H */
