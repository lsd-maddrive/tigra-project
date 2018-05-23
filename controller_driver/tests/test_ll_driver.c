#include <ll_driver_control.h>


void testDriverControlRoutine( void )
{
    llDriverControlInit( );

    drControlSetSteerPower( 250 );
    drControlSetBrakePower( 250 );

    while( true )
    {
        drControlSetMotorDirection( true );
        drControlSetBrakeDirection( true );
        drControlSetSteerDirection( true );
        drControlSetMotorPower( 250 );
    }

}
