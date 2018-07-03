#ifndef INCLUDE_LLD_CONTROL_H_
#define INCLUDE_LLD_CONTROL_H_

#include <common.h>

/*** Variables ***/

typedef int32_t	controlValue_t;

/**
 * @brief   Initialize periphery connected to driver control
 * @note    Stable for repeated calls
 */
void lldControlInit ( void );

/**
 * @brief   Set power for driving motor
 * @param   inputPrc   Motor power value percentage [0 100]
 */
void lldControlSetDrMotorPower ( controlValue_t inputPrc );

/**
 * @brief   Set power for steering motor (via ESC)
 * @param   inputPrc   Motor power value percentage [-100 100]
 */
void lldControlSetSteerPower ( controlValue_t inputPrc );

/**
 * @brief   Set power for braking motor
 * @param   inputPrc   Motor power value percentage [-100 100]
 * @note    power (0, 100]  -> clockwise
 * @note    power [-100, 0) -> counterclockwise
 */
void lldControlSetBrakePower( controlValue_t inputPrc );

/**
 * @brief   Set motor direction
 * @param   lldDrMotorDirection Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetDrMotorDirection ( bool lldDrMotorDirection );



#endif /* INCLUDE_LLD_CONTROL_H_ */
