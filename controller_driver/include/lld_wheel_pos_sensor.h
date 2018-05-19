#ifndef INCLUDE_LLD_WHEEL_POS_SENSOR_H_
#define INCLUDE_LLD_WHEEL_POS_SENSOR_H_

#include <common.h>

/*
 * Hardware description
 * ------------------------------------
 *
 * EXT driver uses input PA8
 * GPT3
 * ICU driver uses input GPT1
 */


/*** Variables ***/
typedef uint32_t   wheelVelocity_t;
typedef uint32_t   wheelPosition_t;


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
wheelPosition_t wheelPosSensorGetPosition ( uint16_t ImpsPerRevQuantity );

/**
 * @ brief                                Gets wheel current velocity value
 *                                        [revolutions per minute (rpm)]
 * @ param [in] ImpsPerRevQuantity        Number of impulses per revolution
 *                                        depends on given sensor
 * @ return                               Current wheel velocity value [rpm]
 *
 */
wheelVelocity_t wheelPosSensorGetVelocity ( uint16_t ImpsPerRevQuantity );

void sendTestInformation ( void );

#endif /* INCLUDE_LLD_WHEEL_POS_SENSOR_H_ */
