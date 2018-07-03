#include <tests.h>
#include <light_unit.h>

typedef enum
{
    LIGHTS_TURN_RIGHT,
    LIGHTS_TURN_LEFT,
    LIGHTS_OFF
} light_states_t;

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

#define stopLight                   PAL_LINE( GPIOE, 2 )



/***  PWM configuration pins    ***/
/***  PE5 - Right Turn Signal   ***/
#define PE5_ACTIVE      PWM_OUTPUT_ACTIVE_HIGH
#define PE5_DISABLE     PWM_OUTPUT_DISABLED
#define rightTurnSignPWMch      0
/***  PE6 - Left Turn Signal    ***/
#define PE6_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE6_DISABLE    PWM_OUTPUT_DISABLED
#define leftTurnSignPWMch      1

#define pwm9LineCh0     PAL_LINE( GPIOE, 5 )
#define pwm9LineCh1     PAL_LINE( GPIOE, 6 )

#define pwm9Freq        40000
#define pwm9Period      20000           // 0.5 s -> 2 Hz

static  PWMDriver       *pwmDriver      = &PWMD9;

/***************************************/
/*** HARDWARE CONFIGURATION ZONE END ***/
/***************************************/


/*** Configuration structures ***/

PWMConfig pwm9conf = {
    .frequency = pwm9Freq,
    .period    = pwm9Period, /* 1/1000 s = 10 ms => 100 Hz
                             * PWM period = period/frequency [s] */
    .callback  = NULL,
    .channels  = {
                  {.mode = PE5_ACTIVE,             .callback = NULL},
                  {.mode = PE6_ACTIVE,             .callback = NULL},
                  {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL},
                  {.mode = PWM_OUTPUT_DISABLED,    .callback = NULL}
                  },
    .cr2        = 0,
    .dier       = 0
};

static bool         isInitialized       = false;

static THD_WORKING_AREA(waTurnSignalThd, 1024);
static THD_FUNCTION(TurnSignalThd, arg)
{
    arg = arg;

    while( 1 )
    {
        if( turnLightState == LIGHTS_TURN_LEFT )
        {
            pwmEnableChannel( pwmDriver, leftTurnSignPWMch, pwm9Period );
            pwmDisableChannel( pwmDriver, rightTurnSignPWMch );
        }
        else if( turnLightState == LIGHTS_TURN_RIGHT )
        {
            pwmEnableChannel( pwmDriver, rightTurnSignPWMch, pwm9Period );
            pwmDisableChannel( pwmDriver, leftTurnSignPWMch);
        }
        else if( turnLightState == LIGHTS_OFF )
        {
            pwmDisableChannel( pwmDriver, leftTurnSignPWMch);
            pwmDisableChannel( pwmDriver, rightTurnSignPWMch );
        }

        chThdSleepMilliseconds( 5 );
    }
}


void lightUnitInit( void )
{
    if ( isInitialized )
          return;

    /*** PWM pins configuration ***/
    palSetLineMode( pwm9LineCh0,  PAL_MODE_ALTERNATE(3) );
    palSetLineMode( pwm9LineCh1,  PAL_MODE_ALTERNATE(3) );

    /*** PAL pins configuration ***/
    palSetLineMode( stopLight, PAL_MODE_OUTPUT_PUSHPULL );

    pwmStart( pwmDriver, &pwm9conf );
    chThdCreateStatic( waTurnSignalThd, sizeof(waTurnSignalThd), NORMALPRIO, TurnSignalThd, NULL );

}

void lightsSetState( int32_t controlSignal )
{

    if( controlSignal <= -20 )
    {
        turnLightState = LIGHTS_TURN_LEFT;
    }
    else if( controlSignal >= 20 )
    {
        turnLightState = LIGHTS_TURN_RIGHT;
    }
    else
    {
        turnLightState = LIGHTS_OFF;
    }

}
