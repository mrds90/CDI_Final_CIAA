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
#ifndef TEST
#include "sapi.h"
#else
#define UART_USB 1
#define uartWriteString(UART_USB, str) printf("%s",str)
#endif
/*========= [PRIVATE MACROS AND CONSTANTS] =====================================*/


#define OPEN_LOOP_CONTROL   1
#define PID_CONTROL         2
#define POLE_PLACEMENT      3
#define POLE_PLACEMENT_OBSERVED 4

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


#define MUL_ELEMENTS(x,y)  ((x)*(y))

#define ERROR(r,x) (2 * (r) - (x))

/*========= [PRIVATE DATA TYPES] ===============================================*/

typedef struct {
    double A[2][2];
    double B[2];
    double C[2];
	double K[2];
	double Ko;
} pole_placement_config_t;

/*========= [TASK DECLARATIONS] ================================================*/

STATIC void CONTROLLER_SquareOpenLoop(void *per);

STATIC void CONTROLLER_PID(void *per);

static void CONTROLLER_PolePlacementControl(void *per);

static void CONTROLLER_PolePlacementControlObserver(void *per);

/*========= [PRIVATE FUNCTION DECLARATIONS] ====================================*/

STATIC int32_t PidRecurrenceFunction(int32_t input);

STATIC double PolePlacementControl(pole_placement_config_t *config, double state[2], double reference);

// STATIC void MatrixMultiply(double **AB, double **A, double **B, int M, int N, int L);

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
    #elif (CONTROL_TASK ==POLE_PLACEMENT)
    OSAL_TASK_Create(&controller_task, CONTROLLER_PolePlacementControl, (void *)(&period), TASK_PRIORITY_NORMAL);
    #elif (CONTROL_TASK ==POLE_PLACEMENT_OBSERVED)
    OSAL_TASK_Create(&controller_task, CONTROLLER_PolePlacementControlObserver, (void *)(&period), TASK_PRIORITY_NORMAL);
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
        input_mv = INTERFACE_ADCRead(1);

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
        input_mv = INTERFACE_ADCRead(1);
        uint32_t input_q15 = (Q15_SCALE(input_mv)) / 3300;
        uint32_t u = PidRecurrenceFunction(ERROR(r[r_index],input_q15));
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

static void CONTROLLER_PolePlacementControl(void *per) {
	pole_placement_config_t pole_placement_config;

    pole_placement_config.K[0] = 0.4881977;
    pole_placement_config.K[1] = 0.6236087;
    pole_placement_config.Ko = 2.115;

    uint8_t period = *((uint8_t *) per);

    static const double r[2] = {2.0, 1.0};

    static uint8_t r_index = 0;
    static uint32_t count = 0;
    
    osal_tick_t last_enter_to_task = OSAL_TASK_GetTickCount();

    #ifndef TEST
    while (TRUE)
    #endif
    {   
        static double state[2];

        state[0] = (double)(INTERFACE_ADCRead(1)/1000.0);
        state[1] = (double)(INTERFACE_ADCRead(2)/1000.0);

        double voltage = PolePlacementControl(&pole_placement_config, state, r[r_index]);
        uint16_t u = (uint16_t)(voltage * 1000);
        
        INTERFACE_DACWriteMv(u);

        count++;
        if (count >= ((period * 1000 / 2) / TS_MS)) {
            count = 0;
            r_index ^= 1;
        }
        static char str[150];
        sprintf(str,"%d,%d,%d,%.d\n", OSAL_TASK_GetTickCount(), (uint16_t)(r[r_index]*1000), u, INTERFACE_ADCRead(1));
        uartWriteString(UART_USB, str);
        
        OSAL_TASK_DelayUntil(&last_enter_to_task, OSAL_MS_TO_TICKS(TS_MS));
    }
}

static void CONTROLLER_PolePlacementControlObserver(void *per) {
	static pole_placement_config_t pole_placement_config ={
       .A = {
           [0] = { 1.24881977, -0.33763913},
           [1] = { 1.,0.}
           },
       .B = {1, 0},
       .C = {0.05233013, 0.03648923},
       .K = {0.4881977, 0.6236087},
       .Ko = 2.115
    };

   pole_placement_config.K[0] = 1.3581298;
   pole_placement_config.K[1] = -0.9386444;
   pole_placement_config.Ko = 1.47229047;
   static const double L[2] = {0.4881977, 0.6236087};
   uint8_t period = *((uint8_t *) per);

   static const double r[2] = {2.0, 1.0};

   static uint8_t r_index = 0;
   static uint32_t count = 0;

   osal_tick_t last_enter_to_task = OSAL_TASK_GetTickCount();

    #ifndef TEST
    while (TRUE)
    #endif
    {
        static double x_est[2] = {0,0};
        static double x_est_tempA[2] = {0,0};
        static double x_est_tempB[2] = {0,0};
        static double x_est_tempC[2] = {0, 0};

        double y = (double)(INTERFACE_ADCRead(1)/1000.0);


        double u = PolePlacementControl(&pole_placement_config, x_est, r[r_index]);

        uint16_t u_dac = (uint16_t)(u * 1000);

        INTERFACE_DACWriteMv(u_dac);

            
        for (int i = 0; i < 2; i++) {
            x_est_tempA[i] = 0;
            for (int j = 0; j < 2; j++) {
                x_est_tempA[i] += MUL_ELEMENTS(pole_placement_config.A[i][j], x_est[j]);
            }
        }

        x_est_tempB[0] = MUL_ELEMENTS(pole_placement_config.B[0], u);
        x_est_tempB[1] = MUL_ELEMENTS(pole_placement_config.B[1], u);
        double cx_est  = MUL_ELEMENTS(pole_placement_config.C[0], x_est[0]) + MUL_ELEMENTS(pole_placement_config.C[1], x_est[1]);
        x_est_tempC[0] = MUL_ELEMENTS(L[0], (y - cx_est));
        x_est_tempC[1] = MUL_ELEMENTS(L[1], (y - cx_est));

        x_est[0] = (x_est_tempA[0] + x_est_tempB[0] + x_est_tempC[0]);
        x_est[1] = (x_est_tempA[1] + x_est_tempB[1] + x_est_tempC[1]);

        count++;
        if (count >= ((period * 1000 / 2) / TS_MS)) {
            count = 0;
            r_index ^= 1;
        }
        static char str[150];
        //    sprintf(str,"%d - %d - %f - %f\n", (uint16_t)(r[r_index]*1000), u, x_est[0],x_est[1]);
        sprintf(str,"%d,%d,%d,%.d\n", OSAL_TASK_GetTickCount(), (uint16_t)(r[r_index]*1000), (int32_t)(u * 1000), ((int32_t)INTERFACE_ADCRead(1)));

        uartWriteString(UART_USB, str);

        OSAL_TASK_DelayUntil(&last_enter_to_task, OSAL_MS_TO_TICKS(TS_MS));
    }
}


/*========= [PRIVATE FUNCTION IMPLEMENTATION] ==================================*/

// STATIC void MatrixMultiply(double **AB, double **A, double **B, int M, int N, int L) {
//     // Representación del producto matricial:
//     //       A[M][N]                B[N][L]               AB[M][L]
//     //  [ A11 A12 ... A1N ]   [ B11 B12 ... B1L ]   [ AB11 AB12 ... AB1L ]
//     //  [ A21 A22 ... A2N ]   [ B21 B22 ... B2L ]   [ AB21 AB22 ... AB2L ]
//     //  [ ... ... ... ... ] X [ ... ... ... ... ] = [ ...  ...  ...  ... ]
//     //  [ AM1 AM2 ... AMN ]   [ BN1 BN2 ... BNL ]   [ ABM1 ABM2 ... ABML ]
//     //
//     // Donde AB[i][j] = suma de (A[i][k] * B[k][j]) para k=0 hasta N-1

//     // Iterar sobre las filas de A
//     for (int i = 0; i < M; i++) {
//         // Iterar sobre las columnas de B
//         for (int j = 0; j < L; j++) {
//             // Inicializar el elemento AB[i][j] a 0
//             AB[i][j] = 0;
//             // Calcular el producto escalar de la fila i de A y la columna j de B
//             for (int k = 0; k < N; k++) {
//                 AB[i][j] += A[i][k] * B[k][j];
//             }
//         }
//     }
// }

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

double PolePlacementControl(pole_placement_config_t *config, double state[2], double reference) {
    #if (CONTROL_TASK == POLE_PLACEMENT_OBSERVED)
    double Ko = 3.89 * config->Ko;
    #else
    double Ko = config->Ko;
    #endif
	return ((Ko * reference) - (config->K[0] * state[0] + config->K[1] * state[1]));
}

/*========= [INTERRUPT FUNCTION IMPLEMENTATION] ================================*/
