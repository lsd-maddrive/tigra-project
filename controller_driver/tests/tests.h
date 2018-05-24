#ifndef TESTS_TESTS_H_
#define TESTS_TESTS_H_

#include <common.h>

/*** Break Sensor tests ***/

/* If defined - simulation is used to generated signals for sensor */
/*
 *	Hardware connection for simulation
 * 	PA4 (DAC) <-> PA7		| Direct connection acts strangely, but it wokrs
 *	Direct control of PA0
 */

#define TEST_BREAK_SENSOR_SIMULATED

/*
 * @brief	Routine of break sensor testing
 * @note	The routine has internal infinite loop
 * @note 	SD7 is used for testing (PE7, PE8)
 */
void testBreakSensorRoutine( void );


/*** Wheel position sensor tests ***/
/*
 *  Hardware connection for simulation
 *  PF13 <-> PF14
 */

#define TEST_WHEEL_POS_SENSOR_SIMULATED
/*
 * @brief   Routine of break sensor testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testWheelPosSensorRoutine( void );


#endif /* TESTS_TESTS_H_ */
