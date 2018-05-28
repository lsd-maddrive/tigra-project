#ifndef INCLUDE_BRAKE_UNIT_CS_H_
#define INCLUDE_BRAKE_UNIT_CS_H_

#include <common.h>

/**
 * @brief       Initialization of brake unit control system
 */
void brakeUnitCSInit( void );

/**
 * @brief               Set brake power reference
 * @param   pressPower  Reference percentage of braking [0; 100]
 */
void brakeUnitCSSetPower( int16_t pressPower );

#endif /* INCLUDE_BRAKE_UNIT_CS_H_ */
