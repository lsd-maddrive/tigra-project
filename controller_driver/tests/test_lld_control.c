#include <tests.h>
#include <lld_control.h>

void testDriverControlRoutine( )
{
    lldControlInit( );

    lldControlSetSteerPower( 250 );
    lldControlSetBrakePower( 250 );
    lldControlSetDrMotorPower( 250 );

    while( true )
    {
        lldControlSetDrMotorDirection( true );
        lldControlSetBrakeDirection( true );
        lldControlSetSteerDirection( true );
        chThdSleepMilliseconds(500);
        lldControlSetDrMotorDirection( false );
        lldControlSetBrakeDirection( false );
        lldControlSetSteerDirection( false );
        chThdSleepMilliseconds(500);
    }
}
