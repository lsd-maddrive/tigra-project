#include <lld_steer_sensors.h>


/* ADC driver related.                                                       */
/*===========================================================================*/

#define adc1NumChannels   1
#define adc1BufDepth      1

static adcsample_t samples1[adc1NumChannels * adc1BufDepth];

static ADCDriver                 *adcSteerSensor    = &ADCD1;
static GPTDriver                 *adcGPT            = &GPTD2;
static GPTNameFunc               *adcGPTfunc        = &gpt2cfg1;


uint16_t lldAdcSteerSensorVal = 0;

/*
 * GPT2 configuration. This timer is used as trigger for the ADC.
 */
static const GPTConfig gpt2cfg1 = {
  .frequency =  100000,
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};


/*
 * ADC streaming callback.
 */

static void adc1cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    adcp = adcp; n = n;

    lldAdcSteerSensorVal = samples1[0];

}

static void adc1Errorcb(ADCDriver *adcp, adcerror_t err) {

    adcp = adcp; err = err;

}

static const ADCConversionGroup adccfg1 = {
  .circular     = true,                     // working mode = looped
  .num_channels = adc1NumChannels,          // number of channels
  .end_cb       = adc1cb,                   // after ADC conversion ends - call this func
  .error_cb     = adc1Errorcb,
  .cr1          = 0,
  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(11),  // Commutated from GPT
  /* 11 means 0b1011, and from RM (p.452) it is GPT2 */
  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),       // for AN10 - 144 samples
  .sqr1         = ADC_SQR1_NUM_CH(adc1NumChannels),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10)
};



/*
 * @brief                   Initialize periphery connected to steering sensor
 */
void lldSteerPositionInit( void )
{
    adcStart( adcSteerSensor, NULL );
    palSetLineMode( LINE_ADC123_IN10, PAL_MODE_INPUT_ANALOG );  // PC0
    adcStartConversion( adcSteerSensor, &adccfg1, samples1, adc1BufDepth);

    gptStart( adcGPT, adcGPTfunc );
    gptStartContinuous( adcGPT, gpt2cfg1.frequency/1000);
}

