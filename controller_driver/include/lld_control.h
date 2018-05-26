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
 * @param   lldMotorPower   Motor power value [0 100]
 */
void lldControlSetDrMotorPower ( uint8_t lldMotorPower );

/*
 * @brief   Set power for steering motor
 * @param   lldSteerPower   Motor power value [0 100]
 */
void lldControlSetSteerPower ( uint8_t lldSteerPower );

/*
 * @brief   Set power for braking motor
 * @param   lldBrakePower   Motor power value [0 100]
 */
void lldControlSetBrakePower ( uint8_t lldBrakePower );

/*
 * @brief   Set motor direction
 * @param   lldDrMotorDirection Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetDrMotorDirection ( bool lldDrMotorDirection );

/*
 * @brief   Set braking motor direction
 * @param   lldBrakeDirection   Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetBrakeDirection ( bool lldBrakeDirection );

/*
 * @brief   Set steering motor direction
 * @param   lldSteerDirection   Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetSteerDirection ( bool lldSteerDirection );



#endif /* INCLUDE_LLD_CONTROL_H_ */
