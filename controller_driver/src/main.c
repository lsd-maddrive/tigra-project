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

#else

    chThdCreateStatic( waThread, sizeof(waThread), NORMALPRIO, Thread, NULL );

    while (true)
    {
        palToggleLine( LINE_LED3 );
        chThdSleepSeconds(1);
    }
    
#endif
}
