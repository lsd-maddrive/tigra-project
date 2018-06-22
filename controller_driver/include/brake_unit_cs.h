#ifndef INCLUDE_BRAKE_UNIT_CS_H_
#define INCLUDE_BRAKE_UNIT_CS_H_

#include <common.h>
#include <lld_brake_sensor.h>
#include <lld_control.h>

#include <controllers.h>

/**
 * @brief   Initialization of brake unit control system
 * @note    Stable for repeated calls
 */
void brakeUnitCSInit( void );

/**
 * @brief   Set brake power reference
 * @param   pressPower  Reference percentage of braking [0; 100]
 * @return 	Control value of controller
 */
controlValue_t brakeUnitCSSetPower( int16_t pressPower );

#endif /* INCLUDE_BRAKE_UNIT_CS_H_ */
