#include <lld_brake_sensor.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

static float    referenceVoltage_mV     = 3300;
static float    sensor_zero_value_mV    = 1650;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Hardware configuration ***/

#define brakeSensorClickLine        PAL_LINE(GPIOA, 0)
#define brakeSensorAnalogInputCh    COMMON_ADC_SEQ1_CH
#define brakeSensorPressedState     PAL_LOW

/*** Hardware configuration end ***/


/*** Driver variables ***/

static bool         isInitialized           = false;
static int32_t      brakePowerValue_mV      = 0;

static uint16_t     sensorMaxVoltage        = 0;
static uint16_t     sensorMinVoltage        = 0;

static float        sensor_k_rate           = 0;

void brakeSensorInit ( void )
{
    if ( isInitialized )
        return;

    commonADC1UnitInit();

    palSetLineMode( brakeSensorClickLine, PAL_MODE_INPUT_PULLUP );

    sensorMaxVoltage      = referenceVoltage_mV;
    sensorMinVoltage      = sensor_zero_value_mV;

    sensor_k_rate           = 100.0 / (sensorMaxVoltage - sensorMinVoltage);

    isInitialized = true;
}

bool brakeSensorIsPressed ( void )
{
    if ( !isInitialized )
        return false;

    return ( palReadLine( brakeSensorClickLine ) == brakeSensorPressedState );
}

int16_t brakeSensorGetPressPower ( void )
{
    if ( !isInitialized )
        return -1;

    brakePowerValue_mV = commonADC1UnitGetValueMV( brakeSensorAnalogInputCh );

    int16_t resultPerc = (brakePowerValue_mV - sensorMinVoltage) * sensor_k_rate;

    return CLIP_VALUE( resultPerc, 0, 100 );
}

int16_t brakeSensorGetVoltage( void )
{
    if ( !isInitialized )
       return -1;

    return brakePowerValue_mV;
}
