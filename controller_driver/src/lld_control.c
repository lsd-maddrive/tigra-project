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

#define pwm1PortCh0     GPIOE
#define pwm1PadCh0      9
#define pwmPortCh1      GPIOE
#define pwmPadCh1       11

#define pwm1Freq        4000000
#define pwm1Period      40000

/*** DAC configuration pins      ***/
#define dacPort         GPIOA
#define dacPad          4

static  PWMDriver       *pwmDriver      = &PWMD1;
static  DACDriver       *dacDriver      = &DACD1;

/*** Direction pins configuration          ***/
/*** F_12 for Driving Wheels Set Direction ***/
#define portMotorDir        GPIOF
#define padMotorDir         12
/*** E_15 for Braking Set Direction        ***/
#define portBrakeDirIN1     GPIOE
#define padBrakeDirIN1      15
#define portBrakeDirIN2     GPIOG
#define padBrakeDirIN2      1
/*** E_3 for Steering Set Direction        ***/
#define portSteerDir        GPIOE
#define padSteerDir         3

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
    palSetPadMode( pwm1PortCh0, pwm1PadCh0, PAL_MODE_ALTERNATE(1) );
    palSetPadMode( pwmPortCh1,  pwmPadCh1,  PAL_MODE_ALTERNATE(1) );

    /*** PAL pins configuration ***/
    palSetPadMode( portMotorDir, padMotorDir, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portBrakeDirIN1, padBrakeDirIN1, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portBrakeDirIN2, padBrakeDirIN2, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portSteerDir, padSteerDir, PAL_MODE_OUTPUT_PUSHPULL );

    /*
    * DAC has two channels
    * Datasheet p69, PA4 - DACout1, PA5 - DACout2
    * Pin configuration for 1st channel
    */
    palSetPadMode( dacPort, dacPad, PAL_MODE_INPUT_ANALOG );

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
 * @param   lldMotorPower   Motor power value [0 100]
 */
void lldControlSetDrMotorPower( int32_t lldMotorPower )
{
    if( lldMotorPower < 0 )
        lldMotorPower = 0;
    else if( lldMotorPower > 100 )
        lldMotorPower = 100;

    uint16_t drDriveDuty = lldMotorPower * speedConvRate + speedLowestDACValue;
    /*
    * Write value to DAC channel
    * Arguments:   <dacDriver>      - pointer to DAC driver
    *              <0>              - channel number (first)
    *              <drMotorPower>   - output value (according to mode/size)
    */

    dacPutChannelX( dacDriver, 0, drDriveDuty );
}

/*
 * @brief   Set power for steering motor
 * @param   lldSteerPower   Motor power value [0 100]
 */
void lldControlSetSteerPower( int32_t lldSteerPower )
{
    if( lldSteerPower > 100 )
        lldSteerPower = 100;
    else if( lldSteerPower < 0 )
        lldSteerPower = 0;

    int16_t  powerInDutyK  =   1;
    int16_t  powerInDutyB  =   0;
    uint16_t drSteerDuty   =   lldSteerPower * powerInDutyK + powerInDutyB;

    pwmEnableChannel( pwmDriver, steerPWMch, drSteerDuty );
}

/*
 * @brief   Set power for braking motor
 * @param   lldBrakePower   Motor power value [-100 100]
 * @note    power (0, 100]  -> clockwise
 * @note    power [-100, 0} -> counterclockwise
 */
void lldControlSetBrakePower( int32_t lldBrakePower )
{

    if( lldBrakePower > 100 )
        lldBrakePower = 100;
    else if( lldBrakePower < -100 )
        lldBrakePower = -100;

    if( lldBrakePower < 0 )
    {
      palSetPad( portBrakeDirIN1, padBrakeDirIN1 );
      palClearPad( portBrakeDirIN2, padBrakeDirIN2 );
    }
    else if( lldBrakePower > 0 )
    {
      palClearPad( portBrakeDirIN1, padBrakeDirIN1 );
      palSetPad( portBrakeDirIN2, padBrakeDirIN2 );
    }

    int16_t  powerInDutyK  =   400;
    int16_t  powerInDutyB  =   0;
    uint16_t drBrakeDuty   =   abs(lldBrakePower) * powerInDutyK + powerInDutyB;

    pwmEnableChannel( pwmDriver, brakePWMch, drBrakeDuty );
}

/*
 * @brief   Set motor direction
 * @param   lldDrMotorDirection Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetDrMotorDirection( bool lldDrMotorDirection )
{
    if(lldDrMotorDirection)
        palSetPad( portMotorDir, padMotorDir );
    else
        palClearPad( portMotorDir, padMotorDir );

}

/*
 * @brief   Set steering motor direction
 * @param   lldSteerDirection   Motor direction true - forward
 *                                              false - backward
 */
void lldControlSetSteerDirection( bool lldSteerDirection )
{
    if(lldSteerDirection)
        palSetPad( portSteerDir, padSteerDir );
    else
        palClearPad( portSteerDir, padSteerDir );
}
