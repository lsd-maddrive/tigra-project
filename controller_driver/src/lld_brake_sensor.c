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

#define brakeSensorClickLine        PAL_LINE(GPIOA, 0)
#define brakeSensorAnalogInputCh    COMMON_ADC_SEQ1_CH

/*** Hardware configuration end ***/


/*** Driver variables ***/

static bool         isInitialized           = false;
static int32_t      brakePowerValue_mV      = 0;

static float        sensor_max_voltage      = 3300;
static float        sensor_min_voltage      = 1650;

static float        sensor_k_rate           = 0;
static float        adcValue2Ref            = 0;


void brakeSensorInit ( void )
{
    if ( isInitialized )
        return;

    commonADC1UnitInit();

    palSetLineMode( brakeSensorClickLine, PAL_MODE_OUTPUT_PUSHPULL );

    // sensor_max_voltage      = sensor_zero_value_mV + sensitivity_rate * sensor_current_lim_A;
    // sensor_min_voltage      = sensor_zero_value_mV;

    sensor_k_rate           = 100.0 / (sensor_max_voltage - sensor_min_voltage);

#if ( COMMON_ADC_RES_CONF == ADC_CR1_6B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 6) - 1);
#elif ( COMMON_ADC_RES_CONF == ADC_CR1_8B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 8) - 1);
#elif ( COMMON_ADC_RES_CONF == ADC_CR1_10B_RESOLUTION )
    adcValue2Ref            = referenceVoltage_mV / ((1 << 10) - 1);
// #elif ( COMMON_ADC_RES_CONF == ADC_CR1_12B_RESOLUTION )
//     adcValue2Ref            = referenceVoltage_mV / ((1 << 12) - 1);
#else
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

    brakePowerValue_mV = commonADC1UnitGetValue( brakeSensorAnalogInputCh ) * adcValue2Ref;

    value = brakePowerValue_mV;
    value = (brakePowerValue_mV - sensor_min_voltage) * sensor_k_rate;
    value = CLIP_VALUE( value, 0, 100 );

    return value;
}

int16_t brakeSensorGetVoltage( void )
{
    if ( !isInitialized )
       return -1;

    return brakePowerValue_mV;
}
