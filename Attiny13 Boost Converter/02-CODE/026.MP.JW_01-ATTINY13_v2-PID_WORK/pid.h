
#ifndef PID_H
#define PID_H

#include "stdint.h"

#define SCALING_FACTOR  128

typedef struct PID_DATA {
    int16_t lastProcessValue;
    int32_t sumError;
    int16_t P_Factor;
    int16_t I_Factor;
    int16_t D_Factor;
    int16_t maxError;
    int32_t maxSumError;
} pidData_t;

// Maximum value of variables                            
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG / 2)

// Boolean values
#define FALSE           0
#define TRUE            1

void pid_Init( int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid );
int16_t pid_Controller( int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st );
void pid_Reset_Integrator( pidData_t *pid_st );

#endif