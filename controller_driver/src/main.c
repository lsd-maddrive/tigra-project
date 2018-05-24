#include <common.h>
#include <tests.h>
#include <chprintf.h>

static THD_WORKING_AREA(waThread, 128);
static THD_FUNCTION(Thread, arg) 
{
    arg = arg;

    while (true)
    {
        chThdSleepSeconds(1);
    }
}

int main(void)
{
    chSysInit();
    halInit();

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BREAK_SENSOR)

    testBreakSensorRoutine();

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
