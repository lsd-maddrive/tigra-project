#ifndef INCLUDE_LLD_WHEEL_POS_SENSOR_H_
#define INCLUDE_LLD_WHEEL_POS_SENSOR_H_

#include <common.h>

/*
 * Hardware description
 * ------------------------------------
 *
 * EXT driver uses input PF13
 * GPT3
 * ICU driver uses input GPT1
 */


/*** Variables ***/
typedef float                   wheelVelocity_t;
typedef uint32_t                wheelPosition_t;
#define ImpsPerRevQuantity      4
#define TimerPeriod             50000

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
 *
 */
wheelPosition_t wheelPosSensorGetPosition ( void );



/**
 * @ brief                           Gets wheel current velocity value
 *                                   [revolutions per minute (rpm)]
 * @ param [in] ImpsPerRevQuantity   Number of impulses per revolution
 *                                   depends on given sensor
 * @ return                          Current wheel velocity value [rpm]
 *
 */
wheelVelocity_t wheelPosSensorGetVelocity ( void );

/*
 * @ brief         Sends test information (gpt counter)
 */
void sendTestInformation ( void );

#endif /* INCLUDE_LLD_WHEEL_POS_SENSOR_H_ */
