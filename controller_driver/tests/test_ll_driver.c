#include <tests.h>
#include <ll_driver_control.h>

void testDriverControlRoutine( )
{
    llDriverControlInit( );

    drControlSetSteerPower( 250 );
    drControlSetBrakePower( 250 );
    drControlSetMotorPower( 250 );

    while( true )
    {
        drControlSetMotorDirection( true );
        drControlSetBrakeDirection( true );
        drControlSetSteerDirection( true );
        chThdSleepMilliseconds(500);
        drControlSetMotorDirection( false );
        drControlSetBrakeDirection( false );
        drControlSetSteerDirection( false );
        chThdSleepMilliseconds(500);
    }
}
