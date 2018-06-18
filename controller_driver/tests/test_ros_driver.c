#include <tests.h>
#include <ros_proto.h>

void testROSDriverRoutine( void )
{
    ros_driver_init();
    ros_driver_start( NORMALPRIO );

    int32_t cntr = 0;

    while ( 1 )
    {
        ros_send_test_i32_msg( cntr++ );

        chThdSleepMilliseconds( 100 );
    }
}
