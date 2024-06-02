/**
 * @file interface.c
 * @author Marcos Dominguez
 *
 * @brief Module for interfacing DAC and ADC with Q15 conversion
 *
 * @version 0.1
 * @date 2024-05-27
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "interface.h"

#define SIMULATED 0
#define REAL      1
#define PLANTA REAL

#if (PLANTA == SIMULATED)
#include "real_world.h"
#elif (PLANTA == REAL)
#include "sapi.h"
#endif

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#define DAC_MAX_MV 3300
#define ADC_MAX_MV 3300

/*========= [PRIVATE DATA TYPES] ===============================================*/

/*========= [TASK DECLARATIONS] ================================================*/

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

#if (PLANTA == REAL)
STATIC uint16_t value10bit[2] = {0,0};
#endif

/*========= [PUBLIC FUNCTION IMPLEMENTATIONS] ==================================*/

void INTERFACE_Init(void) {
    #if (PLANTA == SIMULATED)
    REAL_WORLD_Init();
    #elif (PLANTA == REAL)
    adcConfig( ADC_ENABLE ); /* ADC */
    dacConfig( DAC_ENABLE ); /* DAC */
    #endif
}

/**
 * @brief Write to the DAC with a value in millivolts.
 *
 * @param output_dac_mv Value to write to the DAC in millivolts.
 */
void INTERFACE_DACWriteMv(uint16_t output_dac_mv) {
    // Convert millivolts to Q15
    int32_t value_q15 = (Q15_SCALE(output_dac_mv)) / DAC_MAX_MV; // 9929  19859
    #if (PLANTA == SIMULATED)
    REAL_WORLD_Input(value_q15);
    #elif (PLANTA == REAL)
    dacWrite( DAC, value_q15 >> 5 );
    value10bit[0] = adcRead( CH1 );
    value10bit[1] = adcRead( CH2 );
    #endif
}

/**
 * @brief Read from the ADC and return the value in millivolts.
 *
 * @return uint16_t Value read from the ADC in millivolts.
 */
uint16_t INTERFACE_ADCRead(uint8_t ch) {
    // Read Q15 value from ADC
    uint16_t input_adc_mv = 0; 
    #if (PLANTA == SIMULATED)
    int32_t value_q15 = 0;
    value_q15 = REAL_WORLD_Output();
    // Convert Q15 to millivolts
    input_adc_mv = (value_q15 * ADC_MAX_MV) >> 15;
    #elif (PLANTA == REAL)
    input_adc_mv = (value10bit[ch-1] * ADC_MAX_MV) >> 10;
    #endif

    return input_adc_mv;
}



