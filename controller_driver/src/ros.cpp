#include <ros.h>
#include <ros_proto.h>
#include <common.h>

/***************/
/* SD relative */
/***************/

SerialConfig sdcfg = {
      .speed = 115200,
      .cr1 = 0,
      .cr2 = USART_CR2_LINEN,
      .cr3 = 0
    };

SerialDriver    *ros_sd     = &SD2;
BaseChannel     *ros_sd_ptr = (BaseChannel *)ros_sd;

/**************/
/* ROS things */
/**************/

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

ros::NodeHandle                                 ros_node;

void control_cb( const std_msgs::Int8MultiArray &msg )
{
    if ( msg.data_length != 2 )
        return;

    mainControlSetTask( msg.data[0], msg.data[1] );
}

ros::Subscriber<std_msgs::Int8MultiArray>       topic_control("control_raw", &control_cb);

//=======================================================

/*
 * ROS spin thread - used to receive messages
 */

static THD_WORKING_AREA(waSpinner, 128);
static THD_FUNCTION(Spinner, arg)
{
    (void)arg;
    chRegSetThreadName( "ROS Spinner" );

    while ( true )
    {
        ros_node.spinOnce();

        chThdSleepMilliseconds( 10 );
    }
}

void rosInit( tprio_t prio )
{
    /* Serial driver */
    sdStart( ros_sd, &sdcfg );
    palSetPadMode( GPIOD, 5, PAL_MODE_ALTERNATE(7) );      // TX
    palSetPadMode( GPIOD, 6, PAL_MODE_ALTERNATE(7) );      // RX

    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout( 10 );

    /* ROS publishers */

    /* ROS subscribers */
    ros_node.subscribe( topic_control );

    /* ROS service client */

    /* Main ROS thread */
    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}
