#include <light_unit.h>

/***********************************/
/*** HARDWARE CONFIGURATION ZONE ***/
/***********************************/

#define stopLight           PAL_LINE( GPIOE, 2 )
#define rightTurnLight      PAL_LINE( GPIOH, 1 )
#define leftTurnLight       PAL_LINE( GPIOH, 0 )

/***************************************/
/*** HARDWARE CONFIGURATION ZONE END ***/
/***************************************/

static bool             isInitialized       = false;

#define STOP_LIGHT_MASK             1 << 1
#define RIGHT_LIGHT_MASK            1 << 2
#define LEFT_LIGHT_MASK             1 << 3

static uint8_t      lightMask           = 0;

static uint32_t     leftLightCntr       = 0;
static uint32_t     rightLightCntr      = 0;

static THD_WORKING_AREA(waTurnSignalThd, 1024);
static THD_FUNCTION(TurnSignalThd, arg)
{
    arg = arg;

    while( 1 )
    {
        /* Non-blinking */
        if ( lightMask & STOP_LIGHT_MASK )
            palSetLine( stopLight );
        else
            palClearLine( stopLight );

        if ( lightMask & LEFT_LIGHT_MASK )
        {
            /* Blink each 20th iteration (20 * 50ms = 1s) */
            if ( leftLightCntr++ % 20 )
                palToggleLine( leftTurnLight );
        }
        else
        {
            leftLightCntr = 0;

            palClearLine( leftTurnLight );
        }

        /* Same for right */

        chThdSleepMilliseconds( 50 );
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

    chThdCreateStatic( waTurnSignalThd, sizeof(waTurnSignalThd), NORMALPRIO, TurnSignalThd, NULL );

    isInitialized = true;
}


/**
 * @brief   Set state ONLY for turn light unit depend on control signal value
 */
void turnLightsSetState( light_state_t state )
{
    switch ( state )
    {
        case LIGHTS_TURN_LEFT:
            lightMask |= LEFT_LIGHT_MASK;
            lightMask &= ~RIGHT_LIGHT_MASK;
            break;

        case LIGHTS_TURN_RIGHT:
            lightMask |= RIGHT_LIGHT_MASK;
            lightMask &= ~LEFT_LIGHT_MASK;
            break;

        case LIGHTS_TURN_OFF:
            lightMask &= (~LEFT_LIGHT_MASK & ~RIGHT_LIGHT_MASK);
            break;

        case LIGHTS_BRAKE_ON:
            lightMask |= STOP_LIGHT_MASK;
            break;

        case LIGHTS_BRAKE_OFF:
            lightMask &= ~STOP_LIGHT_MASK;
            break;

        default:
            lightMask = 0;
    }
}
