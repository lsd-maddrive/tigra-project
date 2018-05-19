#include <tests.h>
#include <lld_clutch_lever.h>

void async_func ( void )
{
    palToggleLine( LINE_LED1 );
}

/**
 * @brief   Routine of clutch level of quadrocycle
 * @note    The routine has internal infinite loop
 * @note    Test uses LEDs to check pressing
 */
void testClutchLeverRoutine( void )
{
    clutchLeverInit( async_func );

    while ( 1 )
    {
        chThdSleepSeconds( 1 );
    }
}
