/**
 * @file real_world.c
 * @author Marcos Dominguez
 *
 * @brief Simulate a physical plant
 *
 * @version 0.1
 * @date 2024-05-27
 */

/*========= [DEPENDENCIES] =====================================================*/

#include "real_world.h"
#include "osal_task.h"
#include <string.h>

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

#define NUM_SIZE 2
#define DEN_SIZE 3

// Coeficientes del numerador en Q15
#define NUM0 Q15_SCALE(0.04976845243756167)
#define NUM1 Q15_SCALE(0.035050642374672925)

// Coeficientes del denominador en Q15
#define DEN0 Q15_SCALE(1.0)
#define DEN1 Q15_SCALE(-1.2631799459800208)
#define DEN2 Q15_SCALE(0.34799904079225535)

/*========= [PRIVATE DATA TYPES] ===============================================*/

typedef struct {
    int32_t input;
    int32_t output;
    int32_t input_buffer[NUM_SIZE];
    int32_t output_buffer[DEN_SIZE];
} real_world_t;

/*========= [TASK DECLARATIONS] ================================================*/

STATIC void TaskRealWorld(void *not_used);

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

STATIC int32_t RecurrenceFunction(int32_t input);

/*========= [INTERRUPT FUNCTION DECLARATIONS] ==================================*/

/*========= [LOCAL VARIABLES] ==================================================*/

STATIC real_world_t real_world = {
    .input = 0,
    .output = 0,
    .input_buffer = {[0 ... NUM_SIZE - 1] = 0},
    .output_buffer = {[0 ... DEN_SIZE - 2] = 0},
};


/*========= [STATE FUNCTION POINTERS] ==========================================*/

/*========= [PUBLIC FUNCTION IMPLEMENTATION] ===================================*/

void REAL_WORLD_Init(void) {
    
    static osal_task_t real_world_task = {.name = "real_world"};
    static osal_stack_holder_t real_world_stack[STACK_SIZE_REAL_WORLD];
    static osal_task_holder_t real_world_holder;
    if(real_world_task.task_handler == NULL) {
        OSAL_TASK_LoadStruct(&real_world_task, real_world_stack, &real_world_holder, STACK_SIZE_REAL_WORLD);
        OSAL_TASK_Create(&real_world_task, TaskRealWorld, NULL, TASK_PRIORITY_NORMAL);
    }
}


void REAL_WORLD_Input(int32_t value) {
    real_world.input = value;
}

int32_t REAL_WORLD_Output(void) {
    return real_world.output;
}

void REAL_WORLD_Reset(void) {
    memset(real_world.output_buffer, 0, sizeof(real_world.output_buffer));
    memset(real_world.input_buffer, 0, sizeof(real_world.input_buffer));
    real_world.output = 0;
    real_world.input = 0;
};

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

STATIC void TaskRealWorld(void *not_used) {
    #ifndef TEST
    while (TRUE)
    #endif
    {
        real_world.output = RecurrenceFunction(real_world.input);
        OSAL_TASK_Delay(OSAL_MS_TO_TICKS(5));
    }
}

// Función de recurrencia en tiempo real
STATIC int32_t RecurrenceFunction(int32_t input) {
    // Desplazar valores en el buffer de entrada
    for (int i = NUM_SIZE - 1; i > 0; --i) {
        real_world.input_buffer[i] = real_world.input_buffer[i - 1];
    }
    real_world.input_buffer[0] = input;

    // Calcular la parte del numerador
    int32_t output = 0;
    output += (NUM0 * real_world.input_buffer[0]) >> 15;
    output += (NUM1 * real_world.input_buffer[1]) >> 15;

    // Calcular la parte del denominador
    output -= (DEN1 * real_world.output_buffer[0]) >> 15;
    output -= (DEN2 * real_world.output_buffer[1]) >> 15;

    // Desplazar valores en el buffer de salida
    for (int i = DEN_SIZE - 2; i > 0; --i) {
        real_world.output_buffer[i] = real_world.output_buffer[i - 1];
    }
    real_world.output_buffer[0] = output;

    return output;
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
