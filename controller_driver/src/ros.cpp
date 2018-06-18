#include <ros.h>
#include <ros_proto.h>
#include <common.h>

/*===========================================================================*/
/* SD relative                                                               */
/*===========================================================================*/

SerialConfig sdcfg = {
      .speed = 115200,
      .cr1 = 0,
      .cr2 = USART_CR2_LINEN,
      .cr3 = 0
    };

SerialDriver    *ros_sd     = &SD5;
BaseChannel     *ros_sd_ptr = (BaseChannel *)ros_sd;

/*===========================================================================*/
/* ROS things                                                                */
/*===========================================================================*/

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

ros::NodeHandle                                 ros_node;

std_msgs::Int32                                 i32_test_msg;
ros::Publisher                                  test_topic("test_i32_pub", &i32_test_msg);

bool (*test_srv_cb_func)() = NULL;

void ros_test_srv_set_cb( bool (*cb_func)() )
{
    test_srv_cb_func = cb_func;
}

void test_srv_cb( const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp )
{
    (void)req;

    if ( test_srv_cb_func != NULL )
    {
        bool loc_resp = test_srv_cb_func();

        resp.success = true;
        resp.message = loc_resp ? "On" : "Off";
    }
}

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse> test_srv_serv("test_srv", &test_srv_cb);

//=======================================================

void ros_send_test_i32_msg( int32_t value )
{
    i32_test_msg.data = value;

    test_topic.publish( &i32_test_msg );
}

/*
 * ROS spin thread - used to receive messages
 */

static THD_WORKING_AREA(waSpinner, 128);
static THD_FUNCTION(Spinner, arg)
{
  (void)arg;
  chRegSetThreadName("ROS Spinner");

  while (true)
  {
    ros_node.spinOnce();

    chThdSleepMilliseconds( 10 );
  }
}

void ros_driver_start( tprio_t prio )
{
    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}

void ros_driver_init( void )
{
    /* Serial driver */
    sdStart( ros_sd, &sdcfg );
    palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );      // TX
    palSetPadMode( GPIOD, 2, PAL_MODE_ALTERNATE(8) );      // RX

    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout( 20 );

    /* ROS publishers */
    ros_node.advertise( test_topic );

    /* ROS service client */
    ros_node.advertiseService( test_srv_serv );
}
