#include <tests.h>
#include <lld_steer_sensors.h>


/*** Hardware configuration     ***/

#define adc3NumChannels             1
#define adc3BufDepth                1

static adcsample_t adc3Buffer[adc3NumChannels * adc3BufDepth];

static ADCDriver                    *adcSonar7077      = &ADCD3;
static GPTDriver                    *adcGPT            = &GPTD4;
static  PWMDriver                   *pwmDriver         = &PWMD9;

/***  PWM configuration pins    ***/
/***  PA15 - Trg for Sonar 7077            ***/
#define PE5_ACTIVE          PWM_OUTPUT_ACTIVE_HIGH
#define PE5_DISABLE         PWM_OUTPUT_DISABLED
#define bigSonar7077        0
/***  PB3 - not used            ***/
#define PE6_ACTIVE         PWM_OUTPUT_ACTIVE_HIGH
#define PE6_DISABLE        PWM_OUTPUT_DISABLED


#define pwm9PortCh0      GPIOE
#define pwm9PadCh0       5
#define pwm9Freq         4000000
#define pwm9Period       400000
#define sonarSync        80         // 20 mks (from sensor datasheet)


#define sonar7077AnalogInput         ADC_CHANNEL_IN9


#define sonar7077AnalogLine          PAL_LINE( GPIOF, 3 )

uint16_t lldSonar7077           = 0;


/*** Callback prototype ***/

static void adc3cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{

    adcp = adcp; n = n;

    lldSonar7077              = adc3Buffer[0];

}

/*** Configuration structures ***/

static const GPTConfig gpt4cfg1 = {
  .frequency =  1000000,    // 1 MHz
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

static const ADCConversionGroup adc3cfg = {
  .circular     = true,                     // working mode = looped
  .num_channels = adc3NumChannels,          // number of channels
  .end_cb       = adc3cb,                   // after ADC conversion ends - call this func
  .error_cb     = NULL,
  .cr1          = 0,
  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1100),  // Commutated from GPT
  /* 0b1100, and from RM (p.452) it is GPT4 */
  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
  .smpr1        = ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144),
  .smpr2        = 0,
  .sqr1         = ADC_SQR1_NUM_CH(adc3NumChannels),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(sonar7077AnalogInput)
};

/*
 * @brief                   Synchronization processing of two sensors
 */
PWMConfig pwm9conf = {
    .frequency = pwm9Freq,
    .period    = pwm9Period, /*100 ms => 10 Hz
                              * PWM period = period/frequency [s] */
    .callback  = NULL,
    .channels  = {
                  {.mode = PE5_ACTIVE,              .callback = NULL},
                  {.mode = PE6_DISABLE,             .callback = NULL},
                  {.mode = PWM_OUTPUT_DISABLED,     .callback = NULL},
                  {.mode = PWM_OUTPUT_DISABLED,     .callback = NULL}
                  },
    .cr2        = 0,
    .dier       = 0
};

/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void )
{

    /*** PWM pins configuration ***/
    palSetPadMode( pwm9PortCh0, pwm9PadCh0, PAL_MODE_ALTERNATE(1) );
    /*** ADC pins configuration ***/
    gptStart( adcGPT, &gpt4cfg1 );
    adcStart( adcSonar7077, NULL );
    palSetLineMode( sonar7077AnalogLine,  PAL_MODE_INPUT_ANALOG );

    adcStartConversion( adcSonar7077, &adc3cfg, adc3Buffer, adc3BufDepth);
    pwmStart( pwmDriver, &pwm9conf );
    gptStartContinuous( adcGPT, 100000); // triggering each 100 ms => 10 Hz

}

/*
 * @brief                   Get Adc value of sonar
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSonar7077AdcVal( void )
{

  return lldSonar7077;

}

/*
 * @brief                   Set duty of PWM9 = 20 mks to sync sensor processing
 */
void lldSonarSync( void )
{
     pwmEnableChannel( pwmDriver, bigSonar7077, sonarSync );
}

