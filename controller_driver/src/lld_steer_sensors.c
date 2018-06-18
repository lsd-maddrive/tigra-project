#include <tests.h>
#include <lld_steer_sensors.h>


/*** Hardware configuration     ***/

#define adc1NumChannels             2
#define adc1BufDepth                4

static adcsample_t adc1Buffer[adc1NumChannels * adc1BufDepth];

static ADCDriver                    *adcSteerSensor    = &ADCD1;
static GPTDriver                    *adcGPT            = &GPTD8;


#define steerPosAnalogInput         ADC_CHANNEL_IN10
#define steerPressPowerAnalogInput  ADC_CHANNEL_IN13

#define steerPosAnalogLine          PAL_LINE( GPIOC, 0 )
#define steerPressPowerLine         PAL_LINE( GPIOC, 3 )


uint16_t lldSteerPosVal         = 0;
uint16_t lldSteerPressPowerVal  = 0;

/*** Callback prototype ***/

static void adc1cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    adcp = adcp; n = n;

    if ( buffer != adc1Buffer )
    {
        int i;
        uint32_t sumPos   = 0;
        uint32_t sumPress = 0;
        for ( i = 0; i < adc1BufDepth; i++ )
        {
            sumPos    += adc1Buffer[i*adc1NumChannels];
            sumPress  += adc1Buffer[i*adc1NumChannels+1];
        }

        lldSteerPosVal          = sumPos / adc1BufDepth;
        lldSteerPressPowerVal   = sumPress / adc1BufDepth;
    }
}

/*** Configuration structures ***/

static const GPTConfig gpt8cfg1 = {
  .frequency =  1000000,    // 1 MHz
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

static const ADCConversionGroup adc1cfg = {
  .circular     = true,                     // working mode = looped
  .num_channels = adc1NumChannels,          // number of channels
  .end_cb       = adc1cb,                   // after ADC conversion ends - call this func
  .error_cb     = NULL,
  .cr1          = 0,
  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b0111),  // Commutated from GPT
  /* 11 means 0b0111, and from RM (p.452) it is GPT8 */
  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144) |
                  ADC_SMPR1_SMP_AN13(ADC_SAMPLE_144) ,
  .smpr2        = 0,
  .sqr1         = ADC_SQR1_NUM_CH(adc1NumChannels),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(steerPosAnalogInput) |
                  ADC_SQR3_SQ2_N(steerPressPowerAnalogInput)
};

/*
 * @brief                   Initialize periphery connected to steering sensor
 */
void lldSteerSensorsInit( void )
{

    gptStart( adcGPT, &gpt8cfg1 );
    adcStart( adcSteerSensor, NULL );
    palSetLineMode( steerPosAnalogLine,  PAL_MODE_INPUT_ANALOG );
    palSetLineMode( steerPressPowerLine, PAL_MODE_INPUT_ANALOG );
    adcStartConversion( adcSteerSensor, &adc1cfg, adc1Buffer, adc1BufDepth);

    gptStartContinuous( adcGPT, 2500 ); // triggering each 2.5 ms

}

/*
 * @brief                   Get position of steering
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPosition( void )
{

    return lldSteerPosVal;

}

/*
 * @brief                   Get steering press power
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPressPower( void )
{

    return lldSteerPressPowerVal;

}
