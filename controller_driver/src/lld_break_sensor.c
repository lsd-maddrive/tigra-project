#include <lld_break_sensor.h>

/*** Hardware configuration ***/

#define breakSensorClickLine	PAL_LINE(GPIOB, 0)
#define breakSensolPowerLine    PAL_LINE(GPIOA, 7)
/* ADC channels - DS p65 */
#define breakSensorAnalogInput  ADC_CHANNEL_IN7

static ADCDriver          *breakSensorDriver  = &ADCD2;
static GPTDriver          *adcTriggerDriver   = &GPTD4;

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

#define ADC_CR1_12B_RESOLUTION      0
#define ADC_CR1_10B_RESOLUTION      ADC_CR1_RES_0
#define ADC_CR1_8B_RESOLUTION       ADC_CR1_RES_1
#define ADC_CR1_6B_RESOLUTION       ADC_CR1_RES_0 | ADC_CR1_RES_1

    .cr1            = ADC_CR1_10B_RESOLUTION,
    .cr2            = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b1100),

    /* Input dependent variables */
    .smpr1          = 0,
    .smpr2          = ADC_SMPR2_SMP_AN7(ADC_SAMPLE_480),

    .sqr1           = 0,
    .sqr2           = 0,
    .sqr3           = ADC_SQR3_SQ1_N(breakSensorAnalogInput)
};

static const GPTConfig trigger_cfg = {
    .frequency      =  1000000,
    .callback       =  NULL,
    .cr2            =  TIM_CR2_MMS_1,
    .dier           =  0U
};

/*** Module variables ***/

static uint16_t breakPowerPercent = 0;

/*** ADC callback ***/

static void adc_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp; n = n;

    breakPowerPercent = buffer[0];
}

/*
 * @brief 	Initialize periphery connected to break sensor
 */
void breakSensorInit ( void )
{
    adcStart( breakSensorDriver, NULL );
    palSetLineMode( breakSensolPowerLine, PAL_MODE_INPUT_ANALOG );

    adcStartConversion( breakSensorDriver, &conv_group, adc_buffer, ADC_BUFFER_DEPTH );

    gptStart( adcTriggerDriver, &trigger_cfg );
    gptStartContinuous( adcTriggerDriver, 10000 );
}

/*
 * @brief	Check if break is pressed
 * @return 	true  - break is pressed
 * 			false - break is not pressed
 */
bool breakSensorIsPressed ( void )
{
    bool result = palReadLine( breakSensorClickLine );
	return result;
}

/*
 * @brief	Get press power value
 * @return	[0, 100] - Press power percentage
 * 			< 0 	 - Sensor not initialized
 */
breakPressPower_t breakSensorGetPressPower ( void )
{
	breakPressPower_t value = 0;

	if ( breakSensorIsPressed() )
	{
	    value = breakPowerPercent;
	}

	return value;
}
