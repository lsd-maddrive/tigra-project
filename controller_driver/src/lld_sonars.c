#include <tests.h>
#include <lld_steer_sensors.h>


/*** Hardware configuration     ***/

static GPTDriver                    *trSonarGPT            = &GPTD4;

#define portGreenSonar              GPIOG
#define padGreenSonar               2
#define portBrownSonar              GPIOG
#define padBrownSonar               3
#define portSonar                   GPIOG

static virtual_timer_t reset_vt;
static uint32_t port_arg;

static void reset_cb(void *arg)
{

    palClearPad( portSonar, port_arg );

}

/*
 * @brief                   In callback we turn on sonars one by one
 */
uint8_t countGPT = 0;
static void gpt4cb( GPTDriver *gptp )
{


    countGPT += 1;
    if( countGPT == 1)
    {
      palSetPad( portGreenSonar, padGreenSonar );
      port_arg = padGreenSonar;

    }
    else if( countGPT == 2)
    {
      palSetPad( portBrownSonar, padBrownSonar );
      port_arg = padBrownSonar;

      countGPT = 0;
    }
    chSysLockFromISR();
    chVTSetI(&reset_vt, MS2ST(1), reset_cb, NULL);
    chSysUnlockFromISR();
}

/*** Configuration structures ***/

static const GPTConfig gpt4cfg1 = {
  .frequency =  100000,    // 100 kHz
  .callback  =  gpt4cb,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};


/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void )
{

    /*** PAL pins configuration ***/
    palSetPadMode( portGreenSonar, padGreenSonar, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portBrownSonar, padBrownSonar, PAL_MODE_OUTPUT_PUSHPULL );
    chVTObjectInit(&reset_vt);
    gptStart( trSonarGPT, &gpt4cfg1 );

    gptStartContinuous( trSonarGPT, 10000); // triggering each 100 ms => 10 Hz

}


#if 0
#define adc3NumChannels             2
#define adc3BufDepth                1

static adcsample_t adc3Buffer[adc3NumChannels * adc3BufDepth];

static ADCDriver                    *adcSonar7077      = &ADCD3;
//static GPTDriver                    *adcGPT            = &GPTD4;
static PWMDriver                    *pwm4Driver        = &PWMD4;

/***  PWM configuration pins    ***/
/***  PB8 - Trg for Sonar 7077            ***/
#define PB8_ACTIVE         PWM_OUTPUT_ACTIVE_HIGH
#define PB8_DISABLE        PWM_OUTPUT_DISABLED
#define bigSonar7077       2
/***  PB9 - not used            ***/
#define PB9_ACTIVE         PWM_OUTPUT_ACTIVE_HIGH
#define PB9_DISABLE        PWM_OUTPUT_DISABLED
#define bigSonar7077new    3

#define pwm4PortCh3      GPIOB
#define pwm4PadCh3       8
#define pwm4PortCh4      GPIOB
#define pwm4PadCh4       9

#define pwm4Freq         40000
#define pwm4Period       8000      // 200 ms => 5 Hz
#define sonarSync        4000         // 20 mks


#define sonar7077AnalogInput         ADC_CHANNEL_IN9
#define sonarNew7077AnalogInput      ADC_CHANNEL_IN15


#define sonar7077AnalogLine          PAL_LINE( GPIOF, 3 )
#define sonarNew7077AnalogLine       PAL_LINE( GPIOF, 5 )

uint16_t lldSonar7077           = 0;
uint16_t lldSonar7077new        = 0;


/*** Callback prototype ***/

/*
 * @brief                   write values in sequence (one after each other)
 */
//uint8_t counter = 0;
//static void adc3cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
//{
//
//    adcp = adcp; n = n;
//    counter += 1;
//    if( counter == 1 )
//      lldSonar7077              = adc3Buffer[0];
//    if( counter == 2 )
//    {
//      lldSonar7077new           = adc3Buffer[1];
//      counter = 0;
//    }
//
//}

static void gpt4cb()
{

}

/*** Configuration structures ***/

static const GPTConfig gpt4cfg1 = {
  .frequency =  100000,    // 100 kHz
  .callback  =  gpt4cb,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

//static const ADCConversionGroup adc3cfg = {
//  .circular     = true,                     // working mode = looped
//  .num_channels = adc3NumChannels,          // number of channels
//  .end_cb       = adc3cb,                   // after ADC conversion ends - call this func
//  .error_cb     = NULL,
//  .cr1          = 0,
//  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b0101),  // Commutated from PWM
//  /* 0b0101, and from RM (p.452) it is PWM4 CH 4 */
//  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
//  .smpr1        = ADC_SMPR1_SMP_AN15(ADC_SAMPLE_144),
//  .smpr2        = ADC_SMPR2_SMP_AN9(ADC_SAMPLE_144),
//  .sqr1         = ADC_SQR1_NUM_CH(adc3NumChannels),
//  .sqr2         = 0,
//  .sqr3         = ADC_SQR3_SQ1_N(sonar7077AnalogInput)|
//                  ADC_SQR3_SQ2_N(sonarNew7077AnalogInput)
//};
//
///*
// * @brief                   Synchronization processing of two sensors
// */
//PWMConfig pwm4conf = {
//    .frequency = pwm4Freq,
//    .period    = pwm4Period, /* 100 ms => 10 Hz
//                              * PWM period = period/frequency [s] */
//    .callback  = NULL,
//    .channels  = {
//                  {.mode = PWM_OUTPUT_DISABLED,     .callback = NULL},
//                  {.mode = PWM_OUTPUT_DISABLED,     .callback = NULL},
//                  {.mode = PB8_ACTIVE,              .callback = NULL},
//                  {.mode = PB9_ACTIVE,              .callback = NULL}
//                  },
//    .cr2        = 0,
//    .dier       = 0
//};

/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void )
{

    /*** PWM pins configuration ***/
    palSetPadMode( pwm4PortCh3, pwm4PadCh3, PAL_MODE_ALTERNATE(2) );
    palSetPadMode( pwm4PortCh4, pwm4PadCh4, PAL_MODE_ALTERNATE(2) );
    /*** ADC pins configuration ***/
//    gptStart( adcGPT, &gpt4cfg1 );
    adcStart( adcSonar7077, NULL );
    palSetLineMode( sonar7077AnalogLine,     PAL_MODE_INPUT_ANALOG );
    palSetLineMode( sonarNew7077AnalogLine,  PAL_MODE_INPUT_ANALOG );

    adcStartConversion( adcSonar7077, &adc3cfg, adc3Buffer, adc3BufDepth);
    pwmStart( pwm4Driver, &pwm4conf );

//    gptStartContinuous( adcGPT, 10000); // triggering each 100 ms => 10 Hz

}

/*
 * @brief                   Get Adc value of sonar
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSonar7077AdcVal( uint8_t number )
{
    if( number == 1 )
      return lldSonar7077;
    if( number == 2)
      return lldSonar7077new;
}

/*
 * @brief                   Set duty of PWM9 = 200 mks to sync sensor processing
 */
void lldSonarSync( void )
{

     pwmEnableChannel( pwm4Driver, bigSonar7077,    sonarSync );
     pwmEnableChannel( pwm4Driver, bigSonar7077new, sonarSync );
}
#endif
