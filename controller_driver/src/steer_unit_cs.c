#include <steer_unit_cs.h>


static bool             isInitialized   = false;

#define kP          1
#define kI          0//0.01
#define kD          0

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitCSInit( void )
{
    if ( isInitialized )
        return;

    /* Some initialization sequence */
    lldSteerSensorsInit();
    lldControlInit();

    isInitialized = true;
}

/*
 * @brief   PID implementation
 * @param   steerPower   Reference percentage of steering  [-100 100]
 * @return  controlValue Control value of controller
 */
int32_t     prevControlValue    = 0;
int32_t     counter             = 0;
bool        setNull             = false;
int32_t     limitSpeed          = 0;
int32_t pidCurrentError = 0, pidPreviousError = 0, pidInt = 0, pidDif = 0;

int32_t steerUnitCSSetPower( int16_t steerPower )
{
    int16_t currentSensorValue = 0;

    int32_t controlValue = 0;

    if ( !isInitialized )
          return 0;

    steerPower  = CLIP_VALUE( steerPower, 188, 3885 );

    currentSensorValue = lldSteerPosition();
    pidCurrentError =  steerPower - currentSensorValue;
    pidInt += pidCurrentError;
    pidDif = pidCurrentError - pidPreviousError;

    controlValue = pidCurrentError * kP + pidInt * kI + pidDif * kD;

    if( (controlValue <= 0) && (prevControlValue > 0) )
    {
        palSetLine( LINE_LED1 );
        controlValue = 0;
        setNull = true;
    }

    prevControlValue = controlValue;

    int16_t n = 4;
    if( setNull )
    {
        palSetLine( LINE_LED2 );
        counter += 1;
        if( counter >= 1*n )
        {
          controlValue = 0;
        }
        if( counter >= 2*n )
        {

          palClearLine( LINE_LED2 );
          setNull = false;
          counter = 0;
        }

    }

    if( abs(controlValue) < 10 )
    {
        controlValue = 0;
    }

    /* Set direct power */
    controlValue = CLIP_VALUE( controlValue, -40, 40 );

    pidPreviousError = pidCurrentError;

    lldControlSetSteerPower( controlValue );

    return controlValue;

}
