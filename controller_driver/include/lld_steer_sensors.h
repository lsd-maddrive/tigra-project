#ifndef INCLUDE_LLD_STEER_SENSORS_H_
#define INCLUDE_LLD_STEER_SENSORS_H_

#include <common.h>
#include <chprintf.h>

/*
 * @brief                   Initialize periphery connected to steering sensor
 */
void lldSteerSensorsInit( void );

/*
 * @brief                   Get position of steering
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPosition( void );

/*
 * @brief                   Get steering press power
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPressPower( void );

#endif /* INCLUDE_LLD_STEER_SENSORS_H_ */
