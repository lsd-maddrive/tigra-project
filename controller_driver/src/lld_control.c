#include <tests.h>
#include <lld_control.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

static float    speedLowestVoltage     = 0.8;
/* Can be set or set to 0 to calculate from <speedLowestVoltage> */
static int32_t  speedLowestDACValue    = 0;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Hardware configuration     ***/

/***  PWM configuration pins    ***/
/***  PE9 - Steering            ***/
#define PE9_ACTIVE      PWM_OUTPUT_ACTIVE_HIGH
#define PE9_DISABLE     PWM_OUTPUT_DISABLED
#define steerPWMch      0
/***  PE11 - Braking            ***/
#define PE11_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE11_DISABLE    PWM_OUTPUT_DISABLED
#define brakePWMch      1
/***  PE13, PE14 - not used     ***/
#define PE13_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE13_DISABLE    PWM_OUTPUT_DISABLED
#define PE14_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE14_DISABLE    PWM_OUTPUT_DISABLED

#define pwm1LineCh0     PAL_LINE( GPIOE, 9 )
#define pwm1LineCh1     PAL_LINE( GPIOE, 11 )

#define pwm1Freq        4000000
#define pwm1Period      40000

/*** DAC configuration pins      ***/
#define dacLine         PAL_LINE( GPIOA, 4 )

static  PWMDriver       *pwmDriver      = &PWMD1;
static  DACDriver       *dacDriver      = &DACD1;

/*** Direction pins configuration          ***/
/*** F_12 for Driving Wheels Set Direction ***/
#define lineMotorDir        PAL_LINE( GPIOF, 12 )

/*** E_15 for Braking Set Direction        ***/
#define lineBrakeDirIN1     PAL_LINE( GPIOE, 15 )
#define lineBrakeDirIN2     PAL_LINE( GPIOG, 1 )

static inline void setBrakeDirection ( bool direct )
{
    if ( direct )
    {
        palSetLine( lineBrakeDirIN1 );
        palClearLine( lineBrakeDirIN2 );
    }
    else
    {
        palClearLine( lineBrakeDirIN1 );
        palSetLine( lineBrakeDirIN2 );
    }
}

/*** Configuration structures ***/

PWMConfig pwm1conf = {
    .frequency = pwm1Freq,
    .period    = pwm1Period, /* 1/1000 s = 10 ms => 100 Hz
                             * PWM period = period/frequency [s] */
    .callback  = NULL,
    .channels  = {
                  {.mode = PE9_ACTIVE,      .callback = NULL},
                  {.mode = PE11_ACTIVE,     .callback = NULL},
                  {.mode = PE13_DISABLE,    .callback = NULL},
                  {.mode = PE14_DISABLE,    .callback = NULL}
                  },
    .cr2        = 0,
    .dier       = 0
};

static const DACConfig dac_cfg = {
    /* Initial value of DAC out */
    .init         = 0,
    /*
     * Mode of DAC:
     *      DAC_DHRM_12BIT_RIGHT - 12 bit with right alignment
     *      DAC_DHRM_12BIT_LEFT  - 12 bit with left alignment
     *      DAC_DHRM_8BIT_RIGHT  - 8 bit no alignment (half of dacsample_t [uint16_t] type)
     */
    .datamode     = DAC_DHRM_12BIT_RIGHT,
    /* Direct register set, future used for triggering DAC */
    .cr           = 0
};
/***********************************************************/

static bool         isInitialized       = false;
static float        speedConvRate       = 0.0;

/*
 * @brief   Initialize periphery connected to driver control
 */
void lldControlInit( void )
{
    if ( isInitialized )
        return;

    /*** PWM pins configuration ***/
    palSetLineMode( pwm1LineCh0, PAL_MODE_ALTERNATE(1) );
    palSetLineMode( pwm1LineCh1,  PAL_MODE_ALTERNATE(1) );

    /*** PAL pins configuration ***/
    palSetLineMode( lineMotorDir, PAL_MODE_OUTPUT_PUSHPULL );
    palSetLineMode( lineBrakeDirIN1, PAL_MODE_OUTPUT_PUSHPULL );
    palSetLineMode( lineBrakeDirIN2, PAL_MODE_OUTPUT_PUSHPULL );

    /*
    * DAC has two channels
    * Datasheet p69, PA4 - DACout1, PA5 - DACout2
    * Pin configuration for 1st channel
    */
    palSetLineMode( dacLine, PAL_MODE_INPUT_ANALOG );

    /* Start DAC driver with configuration */
    dacStart( dacDriver, &dac_cfg );

    pwmStart( pwmDriver, &pwm1conf );

    /* Calculate some parameters */

    if ( speedLowestDACValue == 0 )
    {
        speedLowestDACValue = ( 4095 / 3.3 * speedLowestVoltage );
    }

    speedConvRate = (4095 - speedLowestDACValue) / 100.0;

    /* Set initialization flag */

    isInitialized = true;
}

/*
 * @brief   Set power for driving motor
 * @param   inputPrc   Motor power value [0 100]
 */
void lldControlSetDrMotorPower( controlValue_t inputPrc )
{
    inputPrc = CLIP_VALUE( inputPrc, 0, 100 );

    uint16_t drDriveDuty = inputPrc * speedConvRate + speedLowestDACValue;
    /*
    * Write value to DAC channel
    * Arguments:   <dacDriver>      - pointer to DAC driver
    *              <0>              - channel number (first)
    *              <drMotorPower>   - output value (according to mode/size)
    */

    dacPutChannelX( dacDriver, 0, drDriveDuty );
}

/*
 * @brief   Set power for steering motor (via ESC)
 * @param   inputPrc   Motor power value [-100 100]
 */
void lldControlSetSteerPower( controlValue_t inputPrc )
{
    inputPrc = CLIP_VALUE( inputPrc, -100, 100 );
    /*** FIX K and B value !!!! ***/
    int16_t  powerInDutyK  =   1;
    int16_t  powerInDutyB  =   0;
    int32_t  drSteerDuty   =   inputPrc * powerInDutyK + powerInDutyB;
    drSteerDuty = CLIP_VALUE( drSteerDuty, 0, 40000 );
    pwmEnableChannel( pwmDriver, steerPWMch, drSteerDuty );
}

/*
 * @brief   Set power for braking motor
 * @param   inputPrc   Motor power value [-100 100]
 * @note    power (0, 100]  -> clockwise
 * @note    power [-100, 0} -> counterclockwise
 */
void lldControlSetBrakePower( controlValue_t inputPrc )
{
    inputPrc = CLIP_VALUE( inputPrc, -100, 100 );

    setBrakeDirection( inputPrc < 0 );

    int32_t  powerInDutyK  =   400;
    int32_t  powerInDutyB  =   0;
    uint16_t drBrakeDuty   =   abs(inputPrc) * powerInDutyK + powerInDutyB;

    pwmEnableChannel( pwmDriver, brakePWMch, drBrakeDuty );
}

/*
 * @brief   Set motor direction
 * @param   lldDrMotorDirection Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetDrMotorDirection( bool lldDrMotorDirection )
{
    if( lldDrMotorDirection )
        palSetLine( lineMotorDir );
    else
        palClearLine( lineMotorDir );
}
