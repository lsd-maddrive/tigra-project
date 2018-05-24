#include <tests.h>
#include <ll_driver_control.h>

void testDriverControlRoutine( )
{
    llDriverControlInit( );

    drControlSetSteerPower( 250 );
//    palSetLine( LINE_LED2 );
//    drControlSetBrakePower( 250 );

    while( true )
    {
//          drControlSetSteerPower( 250 );
          palSetLine( LINE_LED1 );
//        drControlSetMotorDirection( true );
//        drControlSetBrakeDirection( true );
//        drControlSetSteerDirection( true );
//        drControlSetMotorPower( 250 );
    }

}
