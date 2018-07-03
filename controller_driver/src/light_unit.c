#include <tests.h>
#include <light_unit.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

static light_states_t   turnLightState      = LIGHTS_OFF;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/***********************************/
/*** HARDWARE CONFIGURATION ZONE ***/
/***********************************/

#define stopLight           PAL_LINE( GPIOE, 2 )
#define rightTurnLight      PAL_LINE( GPIOH, 1 )
#define leftTurnLight       PAL_LINE( GPIOH, 0 )


/***************************************/
/*** HARDWARE CONFIGURATION ZONE END ***/
/***************************************/

static bool         isInitialized       = false;

static THD_WORKING_AREA(waTurnLeftThd, 1024);
static THD_FUNCTION(TurnLeftThd, arg)
{
    arg = arg;

    while( 1 )
    {
        if( turnLightState == LIGHTS_TURN_LEFT )
        {
            palToggleLine( LINE_LED1 );
            palToggleLine( leftTurnLight );
            palClearLine( rightTurnLight );
            chThdSleepMilliseconds( 1000 );
        }
        else /* in case of magical error */
        {
            chThdSleepMilliseconds( 50 );
        }
    }
}

static THD_WORKING_AREA(waTurnRightThd, 1024);
static THD_FUNCTION(TurnRightThd, arg)
{
    arg = arg;

    while( 1 )
    {
      if( turnLightState == LIGHTS_TURN_RIGHT )
      {
          palToggleLine( LINE_LED2 );
          palToggleLine( rightTurnLight );
          palClearLine( leftTurnLight );
          chThdSleepMilliseconds( 1000 );
      }
      else /* in case of magical error */
      {
          chThdSleepMilliseconds( 50 );
      }
    }
}

static THD_WORKING_AREA(waTurnOffThd, 1024);
static THD_FUNCTION(TurnOffThd, arg)
{
    arg = arg;

    while( 1 )
    {
        if( turnLightState == LIGHTS_OFF )
        {
            palToggleLine( LINE_LED3 );
            palClearLine( leftTurnLight );
            palClearLine( rightTurnLight );
            chThdSleepMilliseconds( 1000 );
        }
        else /* in case of magical error */
        {
            chThdSleepMilliseconds( 50 );
        }
    }
}


/**
 * @brief   Initialize periphery connected to driver control
 */
void lightUnitInit( void )
{
    if ( isInitialized )
          return;

    /*** PAL pins configuration ***/
    palSetLineMode( leftTurnLight, PAL_MODE_OUTPUT_OPENDRAIN );
    palSetLineMode( rightTurnLight, PAL_MODE_OUTPUT_OPENDRAIN );
    palSetLineMode( stopLight, PAL_MODE_OUTPUT_OPENDRAIN );


    chThdCreateStatic( waTurnLeftThd, sizeof(waTurnLeftThd), NORMALPRIO, TurnLeftThd, NULL );
    chThdCreateStatic( waTurnRightThd, sizeof(waTurnRightThd), NORMALPRIO, TurnRightThd, NULL );
    chThdCreateStatic( waTurnOffThd, sizeof(waTurnOffThd), NORMALPRIO, TurnOffThd, NULL );

}


/**
 * @brief   Set state ONLY for turn light unit depend on control signal value
 */
void turnLightsSetState( light_states_t lightState )
{
    turnLightState = lightState;
    chThdSleepMilliseconds( 10 );

}
//void turnLightsSetState( int32_t controlSignal )
//{
//
//    if( controlSignal <= -20 )
//    {
//        turnLightState = LIGHTS_TURN_LEFT;
//    }
//    else if( controlSignal >= 20 )
//    {
//        turnLightState = LIGHTS_TURN_RIGHT;
//    }
//    else
//    {
//        turnLightState = LIGHTS_OFF;
//    }
//
//}
