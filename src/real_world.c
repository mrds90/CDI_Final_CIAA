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
#include "real_world_filter.h"
#include <string.h>

/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/

/*========= [PRIVATE DATA TYPES] ===============================================*/

typedef struct {
    int32_t input;
    int32_t output;
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
    REAL_WORLD_FILTER_Reset();
    real_world.output = 0;
    real_world.input = 0;
};

/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

STATIC void TaskRealWorld(void *not_used) {
    #ifndef TEST
    while (TRUE)
    #endif
    {
        real_world.output = REAL_WORLD_FILTER_Filter(real_world.input);
        OSAL_TASK_Delay(OSAL_MS_TO_TICKS(5));
    }
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
