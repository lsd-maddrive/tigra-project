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

static PWMDriver        *pwmDriver      = &PWMD1;

/*** Configuration structures ***/

PWMConfig pwm1conf = {
    .frequency = 4000000,   // frequency of timer ticks
    .period    = 5440,      /* 1/1000 s = 1 ms => 1 kHz
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

/*
 * @brief   Initialize periphery connected to driver control
 */
void llDriverControlInit ( void )
{
  palSetPadMode(GPIOE, 9,  PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOE, 11, PAL_MODE_ALTERNATE(2));

  pwmStart( pwmDriver, &pwm1conf );
}


/*
 * @brief   Set power for driving motor
 * @param   drMotorPower    Motor power value [0 100]
 */
void drControlSetMotorPower ( uint8_t drMotorPower )
{

}

/*
 * @brief   Set power for steering motor
 * @param   drSteerPower    Motor power value [0 100]
 */
void drControlSetSteerPower ( uint8_t drSteerPower )
{
    int16_t  powerInDutyK  =    2;
    int16_t  powerInDutyB  =   -2;
    uint16_t drSteerDuty  =   drSteerPower * powerInDuty + powerInDutyB;

    pwmEnableChannel( pwmDriver, steerPWMch, drSteerDuty );
}

/*
 * @brief   Set power for braking motor
 * @param   drBrakePower    Motor power value [0 100]
 */
void drSetBrakePower ( uint8_t drBrakePower )
{
    int16_t  powerInDutyK  =    2;
    int16_t  powerInDutyB  =   -2;
    uint16_t drBkareDuty  =   drBrakePower * powerInDuty + powerInDutyB;

    pwmEnableChannel( pwmDriver, brakePWMch, drSteerDuty );
}

/*
 * @brief   Set motor direction
 * @param   drMotorDirection    Motor direction true - forward
 *                                              false - backward
 */
void drControlSetMotorDirection ( bool drMotorDirection )
{

}
