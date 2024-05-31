/**
 * @file interface.h
 * @author Marcos Dominguez
 *
 * @brief adc dac wrapper.
 *
 * @version 0.1
 * @date 2024-05-27
 */

#ifndef INTERFACE_H
#define INTERFACE_H

#ifdef  __cplusplus
extern "C" {
#endif

/*========= [DEPENDENCIES] =====================================================*/

#include "utils.h"
#include "data_types.h"

/*========= [PUBLIC MACRO AND CONSTANTS] =======================================*/

/*========= [PUBLIC DATA TYPE] =================================================*/

/*========= [PUBLIC FUNCTION DECLARATIONS] =====================================*/

void INTERFACE_Init(void);

void INTERFACE_DACWriteMv(uint16_t output_dac_mv);

uint16_t INTERFACE_ADCRead(void);

#ifdef  __cplusplus
}

#endif

#endif  /* INTERFACE_H */
