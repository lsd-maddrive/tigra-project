#include <lld_brake_sensor.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

static float    referenceVoltage_mV     = 3300;
static float    sensor_current_lim_A    = 4.0;
static float    sensitivity_rate        = ACS712_5AMP_RATE;     // [mV/A]
static float    sensor_zero_value_mV    = 2500;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Hardware configuration ***/

#define brakeSensorClickLine    PAL_LINE(GPIOA, 0)
#define brakeSensolAnalogLine   PAL_LINE(GPIOA, 7)
/* ADC channels - DS p65 */
#define brakeSensorAnalogInput  ADC_CHANNEL_IN7
#define adcResolutionConfig     ADC_CR1_10B_RESOLUTION

static ADCDriver                *brakeSensorDriver  = &ADCD2;
static GPTDriver                *adcTriggerDriver   = &GPTD6;

/*** Hardware configuration end ***/

/*** Callback prototype ***/

static void adc_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n );

/*** Buffer structure ***/

#define ADC_CHANNELS_NUMBER 1
#define ADC_BUFFER_DEPTH    4

static adcsample_t adc_buffer[ADC_CHANNELS_NUMBER * ADC_BUFFER_DEPTH];

/*** Configuration structures ***/

static const ADCConversionGroup  conv_group = {
    .circular       = true,
    .end_cb         = adc_cb,
    .error_cb       = NULL,
    .num_channels   = ADC_CHANNELS_NUMBER,

    .cr1            = adcResolutionConfig,
    .cr2            = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1101),

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

static bool         isInitialized           = false;
static uint16_t     brakePowerValue_mV      = 0;

static float        sensor_max_voltage      = 0;
static float        sensor_min_voltage      = 0;

static float        sensor_k_rate           = 0;
static float        adcValue2Ref            = 0;

/*** ADC callback ***/

static void adc_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp; n = n;

    if ( buffer != adc_buffer )
    {
        int i;
        uint32_t sum = 0;
        for ( i = 0; i < ADC_BUFFER_DEPTH; i++ )
        {
            sum += adc_buffer[i];
        }

        brakePowerValue_mV = (sum / ADC_BUFFER_DEPTH) * adcValue2Ref ;
    }
}

void brakeSensorInit ( void )
{
    if ( isInitialized )
        return;

    adcStart( brakeSensorDriver, NULL );
    palSetLineMode( brakeSensolAnalogLine, PAL_MODE_INPUT_ANALOG );

    adcStartConversion( brakeSensorDriver, &conv_group, adc_buffer, ADC_BUFFER_DEPTH );

    gptStart( adcTriggerDriver, &trigger_cfg );
    /* 2.5ms trigger for filter */
    gptStartContinuous( adcTriggerDriver, 2500 );

    sensor_max_voltage      = sensor_zero_value_mV + sensitivity_rate * sensor_current_lim_A;
    sensor_min_voltage      = sensor_zero_value_mV;

    sensor_k_rate           = 100 / (sensor_max_voltage - sensor_min_voltage);

#if ( adcResolutionConfig == ADC_CR1_6B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 6) - 1);
#elif ( adcResolutionConfig == ADC_CR1_8B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 8) - 1);
#elif ( adcResolutionConfig == ADC_CR1_10B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 10) - 1);
#elif ( adcResolutionConfig == ADC_CR1_12B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 12) - 1);
#endif

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

    value = (brakePowerValue_mV - sensor_min_voltage) * sensor_k_rate;
    value = CLIP_VALUE( value, 0, 100 );

    return value;
}
