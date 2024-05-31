/**
 * @file control.c
 * @author Marcos Dominguez
 *
 * @brief Controller
 *
 * @version 0.1
 * @date 2024-05-27
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "control.h"
#include "interface.h"
#include "task_manager.h"
#include <stdio.h>
#include <string.h>
#include "sapi.h"
/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/


#define OPEN_LOOP_CONTROL   1
#define PID_CONTROL         2

#define CONTROL_TASK PID_CONTROL

#define V_TO_MV(x)  ((x) * 1000)
#define N_SAMPLES (1 << 8)
#define TS_MS         5

#define NUM_SIZE 3
#define DEN_SIZE 3

// Coeficientes del numerador en Q15

#define NUM0 Q15_SCALE(1)
#define NUM1 Q15_SCALE(-1.35569551)
#define NUM2 Q15_SCALE(0.42632345)

// Coeficientes del denominador en Q15
#define DEN0 Q15_SCALE(1)
#define DEN1 Q15_SCALE(-1.21587686)
#define DEN2 Q15_SCALE(0.2865048)

/*========= [PRIVATE DATA TYPES] ===============================================*/


/*========= [TASK DECLARATIONS] ================================================*/

STATIC void CONTROLLER_SquareOpenLoop(void *per);

STATIC void CONTROLLER_PID(void *per);

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

STATIC int32_t PidRecurrenceFunction(int32_t input);

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

STATIC uint16_t input_mv = 0;

/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

void CONTROLLER_Init(void) {
    INTERFACE_Init();
    static uint8_t period = PERIODO_SQUARE;
    static osal_task_t controller_task = {.name = "controller"};
    static osal_stack_holder_t controller_stack[STACK_SIZE_CONTROLLER];
    static osal_task_holder_t controller_holder;
    OSAL_TASK_LoadStruct(&controller_task, controller_stack, &controller_holder, STACK_SIZE_CONTROLLER);
    #if (CONTROL_TASK == OPEN_LOOP_CONTROL)
    OSAL_TASK_Create(&controller_task, CONTROLLER_SquareOpenLoop, (void *)(&period), TASK_PRIORITY_NORMAL);
    #elif (CONTROL_TASK == PID_CONTROL)
    OSAL_TASK_Create(&controller_task, CONTROLLER_PID, (void *)(&period), TASK_PRIORITY_NORMAL);
    #endif

}

STATIC void CONTROLLER_SquareOpenLoop(void *per) {
    uint8_t period = *((uint8_t *) per);
    static const uint16_t output[2] = {V_TO_MV(2), V_TO_MV(1)};
    osal_tick_t last_enter_to_task = OSAL_TASK_GetTickCount();
    static uint8_t r_index = 0;
    static uint32_t count = 0;
    #ifndef TEST
    while (TRUE)
    #endif
    {   
        uint16_t u = output[r_index];
        INTERFACE_DACWriteMv(u);
        input_mv = INTERFACE_ADCRead();

        count++;
        if (count >= ((period * 1000 / 2) / TS_MS)) {
            count = 0;
            r_index ^= 1;
        }
        static char str[150];
        sprintf(str,"%d,%d,%d,%.d\n",OSAL_TASK_GetTickCount(), u, u, input_mv);
        uartWriteString(UART_USB, str);
        OSAL_TASK_DelayUntil(&last_enter_to_task, OSAL_MS_TO_TICKS(TS_MS));
    }
}

STATIC void CONTROLLER_PID(void *per) {
    uint8_t period = *((uint8_t *) per);
    static uint16_t reference = 0;

    static const uint16_t r[2] = {Q15_SCALE(V_TO_MV(2)) / 3300, Q15_SCALE(V_TO_MV(1)) / 3300};

    static uint8_t r_index = 0;
    static uint32_t count = 0;
    osal_tick_t last_enter_to_task = OSAL_TASK_GetTickCount();

    #ifndef TEST
    while (TRUE)
    #endif
    {   
        input_mv = INTERFACE_ADCRead();
        uint32_t input_q15 = (Q15_SCALE(input_mv)) / 3300;
        uint32_t u = PidRecurrenceFunction((2 * r[r_index] - input_q15));
        reference = (r[r_index] * 3300) >> 15;
        u = (u * 3300) >> 15;
        INTERFACE_DACWriteMv(u);

        count++;
        if (count >= ((period * 1000 / 2) / TS_MS)) {
            count = 0;
            r_index ^= 1;
        }
        static char str[150];
        sprintf(str,"%d,%d,%d,%.d\n",OSAL_TASK_GetTickCount(), reference, u, input_mv);
        uartWriteString(UART_USB, str);
        
        OSAL_TASK_DelayUntil(&last_enter_to_task, OSAL_MS_TO_TICKS(TS_MS));
    }
}

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

STATIC int32_t PidRecurrenceFunction(int32_t input) {
// Buffers para mantener el estado
    static int32_t input_buffer[NUM_SIZE] = {[0 ... (NUM_SIZE - 1)] = 0};
    static int32_t output_buffer[DEN_SIZE - 1] = {[0 ... (DEN_SIZE - 2)] = 0};
    // Desplazar valores en el buffer de entrada
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        input_buffer[i] = input_buffer[i - 1];
    }
    input_buffer[0] = input;

    // Calcular la parte del numerador
    int32_t output = 0;
    output += (NUM0 * input_buffer[0]) >> 15;
    output += (NUM1 * input_buffer[1]) >> 15;
    output += (NUM2 * input_buffer[2]) >> 15;

    // Calcular la parte del denominador
    output -= (DEN1 * output_buffer[0]) >> 15;
    output -= (DEN2 * output_buffer[1]) >> 15;

    // Desplazar valores en el buffer de salida
    for (int i = DEN_SIZE - 2; i > 0; --i) {
        output_buffer[i] = output_buffer[i - 1];
    }
    output_buffer[0] = output;

    return output;
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
