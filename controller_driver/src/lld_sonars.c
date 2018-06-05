#include <tests.h>
#include <lld_steer_sensors.h>


/*** Hardware configuration     ***/

#define adc3NumChannels             1
#define adc3BufDepth                1

static adcsample_t adc3Buffer[adc3NumChannels * adc3BufDepth];

static ADCDriver                    *adcSonar7077      = &ADCD3;
static GPTDriver                    *adcGPT            = &GPTD4;


#define sonar7077AnalogInput         ADC_CHANNEL_IN12


#define sonar7077AnalogLine          PAL_LINE( GPIOC, 2 )

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
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void )
{

    gptStart( adcGPT, &gpt4cfg1 );
    adcStart( adcSonar7077, NULL );
    palSetLineMode( sonar7077AnalogLine,  PAL_MODE_INPUT_ANALOG );

    adcStartConversion( adcSonar7077, &adc3cfg, adc3Buffer, adc3BufDepth);

    gptStartContinuous( adcGPT, 10000); // triggering each 10 ms

}

/*
 * @brief                   Get Adc value of sonar
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSonar7077AdcVal()
{

  return lldSonar7077;

}


