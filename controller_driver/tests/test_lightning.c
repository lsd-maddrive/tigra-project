#include <tests.h>
#include <chprintf.h>
#include <light_unit.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};



void testLightningRoutineWorking( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lightUnitInit();

    int32_t controlValue    = 0;
    int32_t counter         = 0;
    bool sireneState        = false;
    chprintf( (BaseSequentialStream *)&SD7, "TEST\n\r" );

    while(1)
    {

        if( ++counter >= 10 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Control Signal:%d, Sirene State: %i\n\r", controlValue, sireneState );
            counter = 0;
        }

        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'q':   // turn on left lights
                turnLightsSetState( LIGHTS_TURN_LEFT );
                break;

            case 'w':   // Negative steer
                turnLightsSetState( LIGHTS_TURN_RIGHT );
                break;
            case 'e':   // turn on brake light
                turnLightsSetState( LIGHTS_BRAKE_ON );
                break;
            case 'r':   // turn off sirene
                turnLightsSetState( LIGHTS_TURN_OFF);
                break;
            case 't':
                turnLightsSetState( LIGHTS_BRAKE_OFF );
                break;
            case 'y':
                sireneSetState( true );
                break;
            case 'u':
                sireneSetState( false );
                break;
            case 'a':
                turnLightsSetState( LIGHTS_BACK_ON );
                break;
            case 's':
                turnLightsSetState( LIGHTS_BACK_OFF );
                break;

            default:
                    ;
        }

        controlValue = CLIP_VALUE( controlValue, -100, 100 );

#ifdef SIMPLE_TEST
        if( ++counter >= 10 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "state: %i\n\r", palReadPad( GPIOE, 6 ) );
            counter = 0;
        }
        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'y':   // Positive steer
                testState = true;
                break;

            case 'u':   // Negative steer
                testState = false;
                break;

            default:
                    ;
        }

        turnOnEverthing( testState );
#endif

        chThdSleepMilliseconds( 10 );
    }
}
