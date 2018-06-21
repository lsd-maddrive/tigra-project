#ifndef INCLUDE_BRAKE_UNIT_CS_H_
#define INCLUDE_BRAKE_UNIT_CS_H_

#include <common.h>
#include <lld_brake_sensor.h>
#include <lld_control.h>

/**
 * @brief   Initialization of brake unit control system
 * @note    Stable for repeated calls
 */
void brakeUnitCSInit( void );

/**
 * @brief   Set brake power reference
 * @param   pressPower  Reference percentage of braking [0; 100]
 */
void brakeUnitCSSetPower( int16_t pressPower );

/**
 * @brief   Get PWM percentage value
 * @return  Value of PWM percentage (0 if not initialized)
 * @note    Must be called after brakeUnitCSSetPower()
 */
int16_t brakeUnitCSGetControl ( void );

#endif /* INCLUDE_BRAKE_UNIT_CS_H_ */
