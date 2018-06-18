#include <tests.h>
#include <ros_proto.h>

bool service_cb ( void )
{
    palToggleLine( LINE_LED1 );

    return ( palReadLine( LINE_LED1 ) == PAL_HIGH );
}

void testROSDriverRoutine( void )
{
    ros_driver_init();
    ros_driver_start( NORMALPRIO );

    ros_test_srv_set_cb( service_cb );

    int32_t cntr = 0;

    while ( 1 )
    {
        ros_send_test_i32_msg( cntr++ );

        chThdSleepMilliseconds( 100 );

        // palToggleLine( LINE_LED2 );
    }
}
