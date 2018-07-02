#ifndef INCLUDE_LLD_STEER_SENSORS_H_
#define INCLUDE_LLD_STEER_SENSORS_H_

#include <common.h>
#include <chprintf.h>

/*
 * @brief   Initialize periphery connected to steering sensor
 */
void lldSteerSensorsInit( void );

/*
 * @brief   Get position of steering
 * @return  Percentage of rotation [-100: 100]
 */
int16_t lldSteerGetPosition( void );

/**
 * @brief 	Check if position has valid state, else break it down
 * @note	Must be called after lldSteerGetPosition()
 * @return 	true - values are valid, everything is initialized
 * 			false - bad values, emergency situation processing required
 */
bool lldSteerSensorsIsValid( void );

/*
 * @brief 	Get steering press power
 * @return  ADC value [0, 4096]
 */
uint16_t lldSteerPressPower( void );

#endif /* INCLUDE_LLD_STEER_SENSORS_H_ */
