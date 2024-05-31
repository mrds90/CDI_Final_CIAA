/**
 * @file identificacion.h
 * @author Marcos Dominguez
 *
 * @brief
 *
 * @version 0.1
 * @date 2024-05-27
 */

#ifndef IDENTIFICACION_H
#define IDENTIFICACION_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "data_types.h"
#include "utils.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define ORDER 2
#define MAX_SAMPLES 200

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

void IDENTIFICACION_Init(void);

#ifdef  __cplusplus
}

#endif

#endif  /* IDENTIFICACION_H */
