#include <lld_brake_sensor.h>

/*** Additional ADC constants ***/

#define ADC_CR1_12B_RESOLUTION      (0)
#define ADC_CR1_10B_RESOLUTION      (ADC_CR1_RES_0)
#define ADC_CR1_8B_RESOLUTION       (ADC_CR1_RES_1)
#define ADC_CR1_6B_RESOLUTION       (ADC_CR1_RES_0 | ADC_CR1_RES_1)

/*** Hardware configuration ***/

#define brakeSensorClickLine    PAL_LINE(GPIOA, 0)
#define brakeSensolAnalogLine   PAL_LINE(GPIOA, 7)
/* ADC channels - DS p65 */
#define brakeSensorAnalogInput  ADC_CHANNEL_IN7
#define adcResolutionConfig     ADC_CR1_10B_RESOLUTION

static ADCDriver                *brakeSensorDriver  = &ADCD2;
static GPTDriver                *adcTriggerDriver   = &GPTD7;

/*** Hardware configuration end ***/

/*** Callback prototype ***/

static void adc_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n );

/*** Buffer structure ***/

#define ADC_CHANNELS_NUMBER 1
#define ADC_BUFFER_DEPTH    1

static adcsample_t adc_buffer[ADC_CHANNELS_NUMBER * ADC_BUFFER_DEPTH];

/*** Configuration structures ***/

static const ADCConversionGroup  conv_group = {
    .circular       = true,
    .end_cb         = adc_cb,
    .error_cb       = NULL,
    .num_channels   = ADC_CHANNELS_NUMBER,

    .cr1            = adcResolutionConfig,
    .cr2            = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1100),

    /* Input dependent variables */
    .smpr1          = 0,
    .smpr2          = ADC_SMPR2_SMP_AN7(ADC_SAMPLE_480),

    .sqr1           = 0,
    .sqr2           = 0,
    .sqr3           = ADC_SQR3_SQ1_N(brakeSensorAnalogInput)
};

static const GPTConfig trigger_cfg = {
    .frequency      =  1000000,         // 1 MHz
    .callback       =  NULL,
    .cr2            =  TIM_CR2_MMS_1,
    .dier           =  0U
};

/*** Module variables ***/

static bool         isInitialized       = false;
static uint16_t     brakePowerPercent   = 0;

#if ( adcResolutionConfig == ADC_CR1_6B_RESOLUTION )
static float        adcValue2Perc       = 100.0 / ((1 << 6) - 1);
#elif ( adcResolutionConfig == ADC_CR1_8B_RESOLUTION )
static float        adcValue2Perc       = 100.0 / ((1 << 8) - 1);
#elif ( adcResolutionConfig == ADC_CR1_10B_RESOLUTION )
static float        adcValue2Perc       = 100.0 / ((1 << 10) - 1);
#elif ( adcResolutionConfig == ADC_CR1_12B_RESOLUTION )
static float        adcValue2Perc       = 100.0 / ((1 << 12) - 1);
#endif

/*** ADC callback ***/

static void adc_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp; n = n;

    brakePowerPercent = buffer[0] * adcValue2Perc;
}

void brakeSensorInit ( void )
{
    if ( isInitialized )
        return;

    adcStart( brakeSensorDriver, NULL );
    palSetLineMode( brakeSensolAnalogLine, PAL_MODE_INPUT_ANALOG );

    adcStartConversion( brakeSensorDriver, &conv_group, adc_buffer, ADC_BUFFER_DEPTH );

    gptStart( adcTriggerDriver, &trigger_cfg );
    /* 10ms trigger */
    gptStartContinuous( adcTriggerDriver, 10000 );

    isInitialized = true;
}

bool brakeSensorIsPressed ( void )
{
    bool result = false;

    if ( !isInitialized )
        return false;

    result = palReadLine( brakeSensorClickLine );

    return result;
}

int16_t brakeSensorGetPressPower ( void )
{
    int16_t value = 0;

    if ( !isInitialized )
        return -1;

    value = brakePowerPercent;

    return value;
}
