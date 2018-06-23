#include <common.h>
#include <lld_steer_sensors.h>


/***    Variable configuration     ***/

#define steerPosAnalogInputCh       COMMON_ADC_SEQ2_CH
#define steerPressAnalogInputCh     COMMON_ADC_SEQ3_CH

uint16_t lldSteerPosVal         = 0;
uint16_t lldSteerPressPowerVal  = 0;


/**
 * @brief                   Initialize periphery connected to steering sensor
 */
void lldSteerSensorsInit( void )
{
    commonADC1UnitInit();
}

/**
 * @brief                   Get position of steering
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPosition( void )
{
    lldSteerPosVal = commonADC1UnitGetValue( steerPosAnalogInputCh );

    return lldSteerPosVal;
}

/**
 * @brief                   Get steering press power
 * @return                  ADC value [0, 4096]
 */
uint16_t lldSteerPressPower( void )
{
    lldSteerPressPowerVal = commonADC1UnitGetValue( steerPressAnalogInputCh );    

    return lldSteerPressPowerVal;
}
