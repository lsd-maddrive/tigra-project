#ifndef INCLUDE_LL_DRIVER_CONTROL_H_
#define INCLUDE_LL_DRIVER_CONTROL_H_

#include <common.h>

#include <stdint.h>




/*
 * @brief   Initialize periphery connected to driver control
 */
void llDriverControlInit ( void );

/*
 * @brief   Set power for driving motor
 * @param   drMotorPower    Motor power value [0 100]
 */
void drControlSetMotorPower ( uint8_t drMotorPower );

/*
 * @brief   Set power for steering motor
 * @param   drSteerPower    Motor power value [0 100]
 */
void drControlSetSteerPower ( uint8_t drSteerPower );

/*
 * @brief   Set power for braking motor
 * @param   drBrakePower    Motor power value [0 100]
 */
void drControlSetBrakePower ( uint8_t drBrakePower );

/*
 * @brief   Set motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetMotorDirection ( bool drMotorDirection );

/*
 * @brief   Set braking motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetBrakeDirection ( bool drBrakeDirection );

/*
 * @brief   Set steering motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetSteerDirection ( bool drSteerDirection );



#endif /* INCLUDE_LL_DRIVER_CONTROL_H_ */
