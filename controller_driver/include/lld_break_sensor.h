#ifndef INCLUDE_LLD_BREAK_SENSOR_H_
#define INCLUDE_LLD_BREAK_SENSOR_H_

#include <common.h>

/*
 * Hardware description
 * ------------------------------------
 * PAL driver uses input PA0
 * ADC2 driver uses input PA7 (AN7) and GPT4 trigger
 */

/*** Variables ***/

typedef int16_t breakPressPower_t;

/*** Prototypes ***/

/*
 * @brief 	Initialize periphery connected to break sensor
 */
void breakSensorInit ( void );

/*
 * @brief	Check if break is pressed
 * @return 	true  - break is pressed
 * 			false - break is not pressed
 */
bool breakSensorIsPressed ( void );

/*
 * @brief	Get press power value
 * @return	[0, 100] - Press power percentage
 * 			< 0 	 - Sensor not initialized
 * @note    Depends on pressed state, get power only if pressed
 */
breakPressPower_t breakSensorGetPressPower ( void );

#endif /* INCLUDE_LLD_BREAK_SENSOR_H_ */
