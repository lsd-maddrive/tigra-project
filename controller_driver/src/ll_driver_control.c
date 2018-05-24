#include <tests.h>
#include <ll_driver_control.h>

/*** Hardware configuration ***/

/***  PE9 - Steering       ***/
#define PE9_ACTIVE      PWM_OUTPUT_ACTIVE_HIGH
#define PE9_DISABLE     PWM_OUTPUT_DISABLED
#define steerPWMch      0
/***  PE11 - Braking        ***/
#define PE11_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE11_DISABLE    PWM_OUTPUT_DISABLED
#define brakePWMch      1
/***  PE13, PE14 - not used ***/
#define PE13_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE13_DISABLE    PWM_OUTPUT_DISABLED
#define PE14_ACTIVE     PWM_OUTPUT_ACTIVE_HIGH
#define PE14_DISABLE    PWM_OUTPUT_DISABLED

static  PWMDriver        *pwmDriver      = &PWMD1;
static  DACDriver        *dacDriver      = &DACD1;

/*** F_12 for Driving Wheels Set Direction ***/
#define portMotorDir    GPIOF
#define padMotorDir     12
/*** E_15 for Braking Set Direction ***/
#define portBrakeDir    GPIOE
#define padBrakeDir     15
/*** E_3 for Steering Set Direction ***/
#define portSteerDir    GPIOE
#define padSteerDir     3

/*** Configuration structures ***/

PWMConfig pwm1conf = {
    .frequency = 4000000,   // frequency of timer ticks
    .period    = 4000,      /* 1/1000 s = 1 ms => 1 kHz
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

/*
 * @brief   Initialize periphery connected to driver control
 */
void llDriverControlInit( void )
{
    /*** PWM pins configuration ***/
    palSetPadMode( GPIOE, 9,  PAL_MODE_ALTERNATE(1) );
    palSetPadMode( GPIOE, 11, PAL_MODE_ALTERNATE(1) );

    /*** PAL pins configuration ***/
    palSetPadMode( portMotorDir, padMotorDir, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portBrakeDir, padBrakeDir, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portSteerDir, padSteerDir, PAL_MODE_OUTPUT_PUSHPULL );

    /*
    * DAC has two channels
    * Datasheet p69, PA4 - DACout1, PA5 - DACout2
    * Pin configuration for 1st channel
    */
    palSetPadMode( GPIOA, 4, PAL_MODE_INPUT_ANALOG );

    /* Start DAC driver with configuration */
    dacStart( dacDriver, &dac_cfg );

    pwmStart( pwmDriver, &pwm1conf );
}

/*
 * @brief   Set power for driving motor
 * @param   drMotorPower    Motor power value [0 100]
 */
void drControlSetMotorPower ( uint8_t drMotorPower )
{
    int16_t  powerInDutyK  =   1;
    int16_t  powerInDutyB  =   0;
    uint16_t drDriveDuty   =   drMotorPower * powerInDutyK + powerInDutyB;
    /*
    * Write value to DAC channel
    * Arguments:   <dacDriver>      - pointer to DAC driver
    *              <0>              - channel number (first)
    *              <drMotorPower>   - output value (according to mode/size)
    */
    // need to fix drMotorPower for DAC
    dacPutChannelX( dacDriver, 0, drDriveDuty );

}

/*
 * @brief   Set power for steering motor
 * @param   drSteerPower    Motor power value [0 100]
 */
void drControlSetSteerPower ( uint8_t drSteerPower )
{
    int16_t  powerInDutyK  =   1;
    int16_t  powerInDutyB  =   0;
    uint16_t drSteerDuty   =   drSteerPower * powerInDutyK + powerInDutyB;

    pwmEnableChannel( pwmDriver, steerPWMch, drSteerDuty );
}

/*
 * @brief   Set power for braking motor
 * @param   drBrakePower    Motor power value [0 100]
 */
void drControlSetBrakePower ( uint8_t drBrakePower )
{
    int16_t  powerInDutyK  =   1;
    int16_t  powerInDutyB  =   0;
    uint16_t drBkareDuty   =   drBrakePower * powerInDutyK + powerInDutyB;

    pwmEnableChannel( pwmDriver, brakePWMch, drBkareDuty );
}

/*
 * @brief   Set motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetMotorDirection( bool drMotorDirection )
{
    if(drMotorDirection)
        palSetPad( portMotorDir, padMotorDir );
    else
        palClearPad( portMotorDir, padMotorDir );

}

/*
 * @brief   Set braking motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetBrakeDirection ( bool drBrakeDirection )
{
    if(drBrakeDirection)
        palSetPad( portBrakeDir, padBrakeDir );
    else
        palClearPad( portBrakeDir, padBrakeDir );
}

/*
 * @brief   Set steering motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetSteerDirection ( bool drSteerDirection )
{
    if(drSteerDirection)
        palSetPad( portSteerDir, padSteerDir );
    else
        palClearPad( portSteerDir, padSteerDir );
}
