#include <tests.h>
#include <lld_clutch_lever.h>

void async_func ( void )
{
    palToggleLine( LINE_LED1 );
}

void testClutchLeverRoutine( void )
{
    clutchLeverInit( async_func );

    while ( 1 )
    {
        chThdSleepSeconds( 1 );
    }
}
