#ifndef TESTS_TESTS_H_
#define TESTS_TESTS_H_

#include <common.h>
/**************************/
/*** Brake Sensor tests ***/
/**************************/

/*  If defined - simulation is used to generated signals for sensor */
/*
 *  Hardware connection for simulation
 *  PA4 (DAC) <-> PA7               | Direct connection acts strangely, but it wokrs
 *  Direct control of PA0
 */

// #define TEST_BRAKE_SENSOR_SIMULATED

/*
 * @brief   Routine of brake sensor testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testBrakeSensorRoutine( void );


/********************************/
/*** Brake Unit Control tests ***/
/********************************/

/**
 * @brief   Routine of brake unit control system
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testBrakeUnitCSRoutine( void );

/**
 * @brief   Routine of brake system integral test
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testBrakeUintOpenedRoutine( void );


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
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 */
void testDriverControlRoutine( void );

/**
 * @brief   Routine of low level driver control testing
 * @note    The routine has internal infinite loop
 * @note    Extended behavior that interacts with user through Serial
 */
void testDriverControlRoutineExt1( void );

/*
 * @brief   Routine of steering sensors testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSteerSensorsWorking( void );

/*
 * @brief   Routine of sonar sensors testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void );


/********************************/
/***Drive Speed Control tests ***/
/********************************/

/*
 * @brief   Routine of motor control system  testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testDriveSpeedCSRoutine( void );

/**
 * @brief   Routine of motor control system test
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testDriveSpeedOpenedRoutine( void );

/*******************************/
/*** Lightning Control tests ***/
/*******************************/

/**
 * @brief   Routine of lightning driver test
 * @note    The routine has internal infinite loop
 */
void testLightningRoutineWorking( void );

/************************/
/*** ROS Driver tests ***/
/************************/

/**
 * @brief   Routine of ROS Driver testing
 * @note    The routine has internal infinite loop
 */
void testROSDriverRoutine( void );

/*******************/
/*** Tools tests ***/
/*******************/

/**
 * @brief   Routine for MATLAB GUI connection and loopback
 * @note    Receives one byte data with next protocol:
 *          -127     - stop sending
 *          127      - start sending
 *          -100:100 - reference data input
 * @note    Sends int16 data valued by control input
 */
void testToolsMatlabSliderRoutine ( void );

/*************************/
/*** Tests application ***/
/*************************/

/**
 * @brief   Routines of tests
 */
static inline void testsRoutines( void )
{
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BRAKE_SENSOR)

    testBrakeSensorRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BRAKE_UNIT_CS)

    testBrakeUnitCSRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_DRIVE_SPEED_CS)

    testDriveSpeedCSRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_DRIVE_SPEED_OPENED)

    testDriveSpeedOpenedRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR)

    testWheelPosSensorRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_CLUTCH_LEVER)

    testClutchLeverRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testDriverControlRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER_SERIAL)

    testDriverControlRoutineSerial();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SONAR)

    testSonarsRoutineWorking();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SHARP)

    testSharpRoutineWorking();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_SENSORS)

    testSteerSensorsWorking();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_ROS_DRIVER)

    testROSDriverRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BRAKE_UNIT_OPENED)

    testBrakeUintOpenedRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LIGHTNING)

    testLightningRoutineWorking();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_TOOLS_MATLAB_SLIDER)

    testToolsMatlabSliderRoutine();

#endif
}

#endif /* TESTS_TESTS_H_ */
