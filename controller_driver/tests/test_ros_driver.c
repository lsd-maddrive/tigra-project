#include <tests.h>
#include <ros_proto.h>

bool service_cb ( void )
{
    palToggleLine( LINE_LED1 );

    return ( palReadLine( LINE_LED1 ) == PAL_HIGH );
}

void testROSDriverRoutine( void )
{
    rosInit( NORMALPRIO );

    rosTestSrvSetCb( service_cb );

    int32_t cntr = 0;

    while ( 1 )
    {
        rosSendTestI32Msg( cntr++ );

        chThdSleepMilliseconds( 100 );

        // palToggleLine( LINE_LED2 );
    }
}
