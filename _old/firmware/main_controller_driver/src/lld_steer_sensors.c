#include <common.h>
#include <lld_steer_sensors.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

static int32_t  steerPosCenterAdc       = 1880;
static int32_t  steerPosMaxAdc          = 2900;
static int32_t  steerPosMinAdc          = 1100;

static int32_t  steerPosValidGap        = 100;

/* Current sensor */
static float    referenceVoltage_mV     = 3300;
static float    sensor_zero_value_mV    = 1650;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Variable configuration ***/

#define steerPosAnalogInputCh       COMMON_ADC_SEQ2_CH
#define steerPressAnalogInputCh     COMMON_ADC_SEQ3_CH

/* ADC value */
uint16_t        lldSteerPosVal              = 0;

static bool     isInitialized               = false;

/*** Calculated in initializtion ***/
static float        steerPosPositiveRate    = 0.0;
static float        steerPosNegativeRate    = 0.0;

/* Current sensor */
static int32_t      steerPowerValue_mV      = 0;

static uint16_t     sensorMaxVoltage        = 0;
static uint16_t     sensorMinVoltage        = 0;

static float        sensor_k_rate           = 0;

/**
 * @brief                   Initialize periphery connected to steering sensor
 */
void lldSteerSensorsInit( void )
{
    commonADC1UnitInit();

    steerPosPositiveRate = 100.0 / ( steerPosMaxAdc - steerPosCenterAdc );
    steerPosNegativeRate = 100.0 / ( steerPosCenterAdc - steerPosMinAdc );

    sensorMaxVoltage      = referenceVoltage_mV;
    sensorMinVoltage      = sensor_zero_value_mV;

    sensor_k_rate           = 100.0 / (sensorMaxVoltage - sensorMinVoltage);

    isInitialized = true;
}

bool lldSteerSensorsIsValid( void )
{
    if ( !isInitialized )
        return false;

    return ( steerPosMinAdc - steerPosValidGap < lldSteerPosVal && lldSteerPosVal < steerPosMaxAdc + steerPosValidGap );
}

int16_t lldSteerGetPositionPrc( void )
{
    if ( !isInitialized )
        return 0;

    /* ADC value - [0; 4095] */
    lldSteerPosVal = commonADC1UnitGetValue( steerPosAnalogInputCh );

    /* Limit just not to break calculation */
    int16_t steerValAdc = CLIP_VALUE( lldSteerPosVal, steerPosMinAdc, steerPosMaxAdc );

    steerValAdc = steerValAdc - steerPosCenterAdc;

    int16_t steerValPerc = steerValAdc > 0 ? steerValAdc * steerPosPositiveRate :
                                             steerValAdc * steerPosNegativeRate;

    /* Just in case */
    steerValPerc = CLIP_VALUE( steerValPerc, -100, 100 );

    return steerValPerc;
}

int16_t lldSteerGetPositionADC( void )
{
    if ( !isInitialized )
        return 0;

    return commonADC1UnitGetValue( steerPosAnalogInputCh );;
}

int16_t lldSteerGetCurrentPrc( void )
{
    if ( !isInitialized )
        return 0;

    steerPowerValue_mV = commonADC1UnitGetValueMV( steerPressAnalogInputCh );    

    int16_t resultPerc = (steerPowerValue_mV - sensorMinVoltage) * sensor_k_rate;

    return CLIP_VALUE( resultPerc, 0, 100 );
}
