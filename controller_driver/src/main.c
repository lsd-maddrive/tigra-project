#include <common.h>
#include <tests.h>
#include <chprintf.h>

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_MASTER)

static THD_WORKING_AREA(waThread, 128);
static THD_FUNCTION(Thread, arg) 
{
    arg = arg;

    while (true)
    {
        chThdSleepSeconds(1);
    }
}

#endif

int main(void)
{
    chSysInit();
    halInit();

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BREAK_SENSOR)

    testBreakSensorRoutine();


#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR)

    testWheelPosSensorRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_DRIVE_SPEED_CS)

    testDriveSpeedCSRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_CLUTCH_LEVER)

    testClutchLeverRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_DRIVER)

    testDriverControlRoutine();


#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_MASTER)

    chThdCreateStatic( waThread, sizeof(waThread), NORMALPRIO, Thread, NULL );

    while (true)
    {
        chThdSleepSeconds(1);
    }
#endif
}
