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


int32_t     prevControlValue    = 0;
int32_t     counter             = 0;
bool        setNull             = false;
int32_t     limitSpeed          = 0;
int32_t     pidCurrentError = 0, pidPreviousError = 0, pidInt = 0, pidDif = 0;

int32_t steerUnitCSSetPosition( int32_t position )
{
    int32_t controlValue = 0;

    if ( !isInitialized )
          return 0;

    position  = CLIP_VALUE( position, -100, 100 );

    int16_t steerPosition = lldSteerGetPosition();

    pidCurrentError =  position - steerPosition;

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
