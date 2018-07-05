#include <common.h>
#include <steer_unit_cs.h>
#include <drive_speed_cs.h>
#include <brake_unit_cs.h>
#include <light_unit.h>
#include <ros_proto.h>

/*** Variables ***/

static int32_t          steerExtTask  = 0;
static int32_t          speedExtTask  = 0;

/* Watchdog timer realization */
#define     CONTROL_SET_TIMEOUT_MS  500

static virtual_timer_t  watchdog_vt;

static void watchdog_cb(void *arg)
{
    arg = arg;

    steerExtTask = speedExtTask = 0;
}

void mainControlSetTask ( int32_t speed, int32_t steer )
{
    steerExtTask = CLIP_VALUE( steer, -100, 100 );
    speedExtTask = CLIP_VALUE( speed, -100, 100 );

    chVTSetI( &watchdog_vt, MS2ST( CONTROL_SET_TIMEOUT_MS ), watchdog_cb, NULL );
}

static THD_WORKING_AREA(waThread, 128);
static THD_FUNCTION(Thread, arg)
{
    arg = arg;
    chRegSetThreadName( "Base control" );

    while (true)
    {
        chThdSleepMilliseconds( 10 );
    }
}

void mainControlTask ( void )
{
    chVTObjectInit( &watchdog_vt );

    chThdCreateStatic( waThread, sizeof(waThread), NORMALPRIO, Thread, NULL );
    mainControlSetTask( 0, 0 );

    /* Main thread */
    while (true)
    {
        palToggleLine( LINE_LED3 );
        chThdSleepSeconds(1);
    }
}

int mainUnitsInit ( void )
{
    steerUnitCSInit();
    driveSpeedCSInit();
    brakeUnitCSInit();

    lightUnitInit();

    rosInit( NORMALPRIO );

    if ( !steerIsEnabled() )
    {
        return EIO;
    }

    return EOK;
}
