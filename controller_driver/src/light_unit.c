#include <light_unit.h>

/***********************************/
/*** HARDWARE CONFIGURATION ZONE ***/
/***********************************/

#define LIGHTS_LED_TEST

#ifndef LIGHTS_LED_TEST
    #define backLight           PAL_LINE( GPIOE, 2 )
    #define stopLight           PAL_LINE( GPIOE, 2 )
    #define rightTurnLight      PAL_LINE( GPIOH, 1 )
    #define leftTurnLight       PAL_LINE( GPIOH, 0 )
#else
    #define backLight           LINE_LED1
    #define stopLight           LINE_LED1
    #define rightTurnLight      LINE_LED2
    #define leftTurnLight       LINE_LED3
#endif

/***************************************/
/*** HARDWARE CONFIGURATION ZONE END ***/
/***************************************/

static bool             isInitialized       = false;

#define STOP_LIGHT_BIT          (1 << 1)
#define RIGHT_LIGHT_BIT         (1 << 2)
#define LEFT_LIGHT_BIT          (1 << 3)
#define BACK_LIGHT_BIT          (1 << 4)

static uint8_t      lightMask           = 0;

#define SET_LIGHT_BIT(bit)      (lightMask |= (bit))
#define CLR_LIGHT_BIT(bit)      (lightMask &= ~(bit))

static uint32_t     leftLightCntr       = 0;
static uint32_t     rightLightCntr      = 0;

static THD_WORKING_AREA(waTurnSignalThd, 1024);
static THD_FUNCTION(TurnSignalThd, arg)
{
    arg = arg;
    chRegSetThreadName( "Lights" );

    while( 1 )
    {
        /* Non-blinking */
        if ( lightMask & STOP_LIGHT_BIT )
            palSetLine( stopLight );
        else
            palClearLine( stopLight );


        if ( lightMask & BACK_LIGHT_BIT )
            palSetLine( backLight );
        else
            palClearLine( backLight );


        if ( lightMask & LEFT_LIGHT_BIT )
        {
            /* Blink each 20th iteration (20 * 50ms = 1s) */
            if ( leftLightCntr++ % 20 == 0 )
                palToggleLine( leftTurnLight );
        }
        else
        {
            leftLightCntr = 0;
            palClearLine( leftTurnLight );
        }


        /* Same for right */
        if ( lightMask & RIGHT_LIGHT_BIT )
        {
            /* Blink each 20th iteration (20 * 50ms = 1s) */
            if ( rightLightCntr++ % 20 == 0 )
                palToggleLine( rightTurnLight );
        }
        else
        {
            rightLightCntr = 0;
            palClearLine( rightTurnLight );
        }

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
#ifndef LIGHTS_LED_TEST
    palSetLineMode( leftTurnLight, PAL_MODE_OUTPUT_OPENDRAIN );
    palSetLineMode( rightTurnLight, PAL_MODE_OUTPUT_OPENDRAIN );
    palSetLineMode( stopLight, PAL_MODE_OUTPUT_OPENDRAIN );
    palSetLineMode( backLight, PAL_MODE_OUTPUT_OPENDRAIN );
#endif

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
            SET_LIGHT_BIT( LEFT_LIGHT_BIT );
            CLR_LIGHT_BIT( RIGHT_LIGHT_BIT );
            break;

        case LIGHTS_TURN_RIGHT:
            SET_LIGHT_BIT( RIGHT_LIGHT_BIT );
            CLR_LIGHT_BIT( LEFT_LIGHT_BIT );
            break;

        case LIGHTS_TURN_OFF:
            CLR_LIGHT_BIT( LEFT_LIGHT_BIT | RIGHT_LIGHT_BIT );
            break;

        case LIGHTS_BRAKE_ON:
            SET_LIGHT_BIT( STOP_LIGHT_BIT );
            break;

        case LIGHTS_BRAKE_OFF:
            CLR_LIGHT_BIT( STOP_LIGHT_BIT );
            break;

        case LIGHTS_BACK_ON:
            SET_LIGHT_BIT( BACK_LIGHT_BIT );
            break;

        case LIGHTS_BACK_OFF:
            CLR_LIGHT_BIT( BACK_LIGHT_BIT );
            break;

        default:
            lightMask = 0;
    }
}
