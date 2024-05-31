/**
 * @file control.h
 * @author Marcos Dominguez
 *
 * @brief Controller
 *
 * @version 0.1
 * @date 2024-05-27
 */


#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "data_types.h"
#include "utils.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

#define PERIODO_SQUARE 1
/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

void CONTROLLER_Init(void);

#ifdef  __cplusplus
}

#endif

#endif  /* CONTROLLER_H */
