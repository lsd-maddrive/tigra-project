#ifndef INCLUDE_LLD_WHEEL_POS_SENSOR_H_
#define INCLUDE_LLD_WHEEL_POS_SENSOR_H_

#include <common.h>
#include <chprintf.h>
/*
 * Hardware description
 * ------------------------------------
 *
 * EXT driver uses input PF13
 * GPT3 used for time intervals between wheel position sensor fronts measurement
 */

/*** Variables ***/
typedef float                   wheelVelocity_t;
typedef float                   wheelPosition_t;
/* Wheel position sensor configuration
 * impulse quantity per revolution*/
#define ImpsPerRevQuantity      4


/*** Prototypes ***/

/*
 * @brief   Initialize periphery connected to wheel position sensor
 */
void wheelPosSensorInit (void);


/**
 * @ brief                           Gets wheel current position value
 *                                   [revolutions]
 * @ param[in] ImpsPerRevQuantity    Number of impulses per revolution
 *                                   depends on given sensor
 * @ return                          Current wheel position value [revolutions]
 */
wheelPosition_t wheelPosSensorGetPosition ( void );


/**
 * @ brief                          Gets wheel current velocity value
 *                                  [revolutions per minute (rpm)]
 * @ param[in] ImpsPerRevQuantity   Number of impulses per revolution
 *                                  depends on given sensor
 * @ return  >=0                    Current wheel velocity value [rpm]
 *           -1                     Sensor is not initialized
 * @ note                           w = dx/dt; dx = (1/ImpsPerRevQuantity) [revolutions]
 *                                  dt = time interval [ticks] / ( timer frequency[ticks/s] * 60 ) [min]
 *                                  w = ((60 / ImpsPerRevQuantity) * timer frequency)/ time interval
 */
wheelVelocity_t wheelPosSensorGetVelocity ( void );


/*
 * @ brief         Sends test information (gpt counter)
 */
void sendTestInformation ( void );


#endif /* INCLUDE_LLD_WHEEL_POS_SENSOR_H_ */
