#ifndef INCLUDE_LLD_BRAKE_SENSOR_H_
#define INCLUDE_LLD_BRAKE_SENSOR_H_

#include <common.h>

/*** Variables ***/


/*** Prototypes ***/

/*
 * @brief   Initialize periphery connected to brake sensor
 * @note    Stable for repeated calls
 */
void brakeSensorInit ( void );

/*
 * @brief   Check if brake is pressed
 * @return  true    brake is pressed
 *          false   brake is not pressed (or not initialized)
 */
bool brakeSensorIsPressed ( void );

/*
 * @brief   Get press power value
 * @return  [0, 100]        Percentage of reference voltage (3V3)
 *          < 0             Sensor is not initialized
 * @note                    Depends on pressed state, get power only if pressed
 */
int16_t brakeSensorGetPressPower ( void );

#endif /* INCLUDE_LLD_BRAKE_SENSOR_H_ */
