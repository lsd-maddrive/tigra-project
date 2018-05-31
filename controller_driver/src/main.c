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

#if (MAIN_PROGRAM_ROUTINE != PROGRAM_ROUTINE_MASTER)

    testsRoutines();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_CLUTCH_LEVER)

    testClutchLeverRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LLD_CONTROL)

    testDriverControlRoutine();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_STEER_SENSORS)

    testSteerSensors();

#elif (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_MASTER)

    chThdCreateStatic( waThread, sizeof(waThread), NORMALPRIO, Thread, NULL );

    while (true)
    {
        chThdSleepSeconds(1);
    }
#endif
}
