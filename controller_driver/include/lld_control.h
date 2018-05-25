#ifndef INCLUDE_LLD_CONTROL_H_
#define INCLUDE_LLD_CONTROL_H_

#include <common.h>

#include <stdint.h>




/*
 * @brief   Initialize periphery connected to driver control
 */
void lldControlInit ( void );

/*
 * @brief   Set power for driving motor
 * @param   drMotorPower    Motor power value [0 100]
 */
void lldControlSetDrMotorPower ( uint8_t lldMotorPower );

/*
 * @brief   Set power for steering motor
 * @param   drSteerPower    Motor power value [0 100]
 */
void lldControlSetSteerPower ( uint8_t lldSteerPower );

/*
 * @brief   Set power for braking motor
 * @param   drBrakePower    Motor power value [0 100]
 */
void lldControlSetBrakePower ( uint8_t lldBrakePower );

/*
 * @brief   Set motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetDrMotorDirection ( bool lldDrMotorDirection );

/*
 * @brief   Set braking motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetBrakeDirection ( bool lldBrakeDirection );

/*
 * @brief   Set steering motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetSteerDirection ( bool lldSteerDirection );



#endif /* INCLUDE_LLD_CONTROL_H_ */
