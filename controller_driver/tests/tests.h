#ifndef TESTS_TESTS_H_
#define TESTS_TESTS_H_

#include <common.h>
/**************************/
/*** Break Sensor tests ***/
/**************************/

/*  If defined - simulation is used to generated signals for sensor */
/*
 *  Hardware connection for simulation
 *  PA4 (DAC) <-> PA7               | Direct connection acts strangely, but it wokrs
 *  Direct control of PA0
 */

#define TEST_BREAK_SENSOR_SIMULATED

/*
 * @brief   Routine of break sensor testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testBreakSensorRoutine( void );

/***********************************/
/*** Wheel position sensor tests ***/
/***********************************/

/*
 *  Hardware connection for simulation
 *  PF13 <-> PF14 (pulses)
 */

#define TEST_WHEEL_POS_SENSOR_SIMULATED
/*
 * @brief   Routine of wheel position sensor testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testWheelPosSensorRoutine( void );

/**************************/
/*** Clutch lever tests ***/
/**************************/

/**
 * @brief   Routine of clutch level of quadrocycle
 * @note    The routine has internal infinite loop
 * @note    Test uses LEDs to check pressing
 */
void testClutchLeverRoutine( void );

/****************************/
/*** Driver Control tests ***/
/****************************/

/*
 * @brief   Routine of low lovel driver control testing
 * @note    The routine has internal infinite loop
 */
void testDriverControlRoutine( void );

/*************************/
/*** Tests application ***/
/*************************/

/**
 * @brief   Routines of tests
 */
static inline void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BREAK_SENSOR)

    testBreakSensorRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR)

    testWheelPosSensorRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_CLUTCH_LEVER)

    testClutchLeverRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testDriverControlRoutine();

#endif
}

#endif /* TESTS_TESTS_H_ */
