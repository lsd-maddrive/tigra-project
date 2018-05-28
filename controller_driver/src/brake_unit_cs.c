#include <lld_control.h>
#include <lld_brake_sensor.h>
#include <brake_unit_cs.h>

void brakeUnitCSInit( void )
{
    /* Some initialization sequence */
}

void brakeUnitCSSetPower( int16_t pressPower )
{
    pressPower = CLIP_VALUE( pressPower, 0, 100 );
}
