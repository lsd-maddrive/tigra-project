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
#define     CONTROL_SET_TIMEOUT_MS  2000
#define     MODE_SET_TIMEOUT_MS     500

#define BASE_MODE_IDLE      0
#define BASE_MODE_PREPARE   1
#define BASE_MODE_START     2

static uint8_t          currentMode   = BASE_MODE_IDLE;

static virtual_timer_t  watchdog_vt;
static virtual_timer_t  watchdog_mode;

static void watchdog_cb(void *arg)
{
    arg = arg;

    steerExtTask = speedExtTask = 0;
}

void mainControlSetSpeed ( int32_t speed )
{
    // chprintf( (BaseSequentialStream *)&SD7, "Set spd: %d\n", speed );

    speedExtTask = CLIP_VALUE( speed, -100, 100 );

    chVTSet( &watchdog_vt, MS2ST( CONTROL_SET_TIMEOUT_MS ), watchdog_cb, NULL );
}

void mainControlSetSteer ( int32_t steer )
{
    // chprintf( (BaseSequentialStream *)&SD7, "Set str: %d\n", steer );

    steerExtTask = CLIP_VALUE( steer, -80, 80 );

    // chVTSet( &watchdog_vt, MS2ST( CONTROL_SET_TIMEOUT_MS ), watchdog_cb, NULL );
}

static THD_WORKING_AREA(waThread, 128);
static THD_FUNCTION(Thread, arg)
{
    arg = arg;
    chRegSetThreadName( "Base control" );

    uint32_t    printCntr = 0;

    while ( true )
    {
        if ( currentMode == BASE_MODE_START )
        {
            driveSpeedControl( speedExtTask );
            steerUnitCSSetPosition( steerExtTask );
        }

        if ( currentMode == BASE_MODE_PREPARE || currentMode == BASE_MODE_START )
        {
            sireneSetState( true );
        }
        else
        {
            steerUnitCSSetPosition( 0 );
            sireneSetState( false );
        }

        if ( printCntr++ > 10 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Task speed: %d / steer: %d / mode: %d\n", 
                                                    speedExtTask, steerExtTask, currentMode );
            printCntr = 0;
        }

        chThdSleepMilliseconds( 10 );
    }
}

void mainControlTask ( void )
{
    chVTObjectInit( &watchdog_vt );

    chThdCreateStatic( waThread, sizeof(waThread), NORMALPRIO + 1, Thread, NULL );

    /* Main thread */
    while ( true )
    {
        chThdSleepSeconds(1);
    }
}

int mainUnitsInit ( void )
{
    /*** Debug ***/
    static const SerialConfig sdcfg = {
      .speed = 115200,
      .cr1 = 0, .cr2 = 0, .cr3 = 0
    };

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    steerUnitCSInit();
    driveSpeedCSInit();
    brakeUnitCSInit();

    lightUnitInit();

    rosInit( NORMALPRIO );

    // if ( !steerIsEnabled() )
    // {
    //     return EIO;
    // }

    return EOK;
}

mainControlInfo_t mainControlGetInfo( void )
{
    mainControlInfo_t   info;

    info.speedTask  = speedExtTask;
    info.steerTask  = steerExtTask;

    info.mode       = currentMode;

    return info;
}

static void watchdog_mode_cb(void *arg)
{
    arg = arg;

    /* TODO - mode timeout processing */


}

/**
 * @brief       set working mode
 */
void mainControlSetMode( uint8_t mode )
{
    chprintf( (BaseSequentialStream *)&SD7, "Mode: %d\n", mode );

    if ( mode == BASE_MODE_START && currentMode == BASE_MODE_IDLE )
    {
        chprintf( (BaseSequentialStream *)&SD7, "Bad mode <<< %d\n", mode );
    }
    else
    {
        currentMode = mode;
    }

    chVTSet( &watchdog_mode, MS2ST( MODE_SET_TIMEOUT_MS ), watchdog_mode_cb, NULL );
}
