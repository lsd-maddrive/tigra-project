#include <tests.h>

#define testOD                      PAL_LINE( GPIOE, 2 )

void testLightningRoutineWorking( void )
{

    palSetLineMode( testOD, PAL_MODE_OUTPUT_OPENDRAIN );

    while(1)
    {

        palToggleLine( testOD );
        palToggleLine( LINE_LED1 );
        chThdSleepMilliseconds( 1000 );

    }

}
