#pragma once

#include <stdlib.h>
#include <stdint.h>

/* Independent module, no need of ChibiOS env */

/*** Variables ***/
typedef float       controllerRate_t;
typedef float       controllerError_t;
typedef float       controllerRensponse_t;

/* Configuration - PID controller parameters*/
typedef struct 
{
    /* User defined parameters */
    controllerRate_t    kp;
    controllerRate_t    ki;
    controllerRate_t    kd;
    float               integrLimit;
	controllerError_t   err;

    /* Service parameters */
    float               integrSum;
    controllerError_t   prevErr;

} PIDControllerContext_t;


/*** Prototypes ***/

/**
 * @brief   PID controller context initialization
 * @params  ctx      PID controller context
 * @note    Check user defined parameters
 */
void PIDControlInit ( PIDControllerContext_t *ctx );

/**
 * @brief       Control system law realization. PID controller *
 * @params[in]  ctx     PID controller context
 * @params[out] Controller output <controllerRensponse_t>
 */
controllerRensponse_t PIDControlResponse ( PIDControllerContext_t *ctx );
