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

ros::NodeHandle                                 ros_node;

//#define CONFIG_IS_READY

#ifdef CONFIG_IS_READY
void (*g_cb_func)(uint16_t speed, uint16_t steer) = NULL;

// Example callback
void topic_cb( const std_msgs::UInt16MultiArray &msg )
{
    if ( msg.data_length != 2 )
        return;

    if ( g_cb_func != NULL )
    {
        g_cb_func( msg.data[0], msg.data[1] );
    }
}

void led_cb( const std_msgs::UInt8 &msg )
{
  palToggleLine( LINE_LED1 );
}

std_msgs::UInt16MultiArray                      u16_arr_msg;
std_msgs::Int32                                 i32_odom_msg;
std_msgs::UInt8                                 u8_mode_msg;

ros::Publisher                                  topic_odom("odom_raw", &i32_odom_msg);
ros::Publisher                                  topic_ranges("ranges_raw", &u16_arr_msg);
ros::Publisher                                  topic_mode("mode", &u8_mode_msg);
ros::Subscriber<std_msgs::UInt8>                topic_led("led", &led_cb);
ros::Subscriber<std_msgs::UInt16MultiArray>     topic_control("control_raw", &topic_cb);
#endif

std_msgs::Int32                                 i32_test_msg;
ros::Publisher                                  test_topic("test_i32_pub", &i32_test_msg);

bool (*test_srv_cb_func)( int32_t value ) = NULL;

void ros_test_srv_set_cb( bool (*cb_func)( int32_t value ) )
{
    test_srv_cb_func = cb_func;
}

void test_srv_cb( const std_msgs::Int32 &req, std_msgs::Bool &resp )
{
    resp.data = false;

    if ( test_srv_cb_func != NULL )
    {
        bool loc_resp = test_srv_cb_func( req.data );
        resp.data = loc_resp;
    }
}

ros::ServiceServer<std_msgs::Int32, std_msgs::Bool> test_srv_serv("test_srv", &test_srv_cb);

//=======================================================
#ifdef CONFIG_IS_READY
void ros_driver_set_control_cb( void (*cb_func)(uint16_t speed, uint16_t steer) )
{
  g_cb_func = cb_func;
}

void ros_driver_send_rangefinders( uint16_t *data, uint32_t data_size )
{
  u16_arr_msg.data          = data;
  u16_arr_msg.data_length   = data_size;

  topic_ranges.publish(&u16_arr_msg);
}

void ros_driver_send_mode( uint8_t m_mode )
{
  u8_mode_msg.data = m_mode;

  topic_mode.publish(&u8_mode_msg);
}

void ros_driver_send_odometry( int32_t counter )
{
  i32_odom_msg.data         = counter;

  topic_odom.publish(&i32_odom_msg);
}
#endif

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
    palSetPadMode( GPIOB, 13, PAL_MODE_ALTERNATE(8) );      // TX
    palSetPadMode( GPIOB, 12, PAL_MODE_ALTERNATE(8) );      // RX

    /* ROS setup */
    ros_node.initNode();
    ros_node.setSpinTimeout( 20 );

    /* ROS publishers */
    ros_node.advertise( test_topic );

    /* ROS service client */
    ros_node.advertiseService( test_srv_serv );

#ifdef CONFIG_IS_READY

    /* ROS subscribers */
    ros_node.subscribe(topic_led);
    ros_node.subscribe(topic_control);

#endif
}
