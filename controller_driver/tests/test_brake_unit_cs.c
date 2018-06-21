#include <tests.h>
#include <chprintf.h>
#include <brake_unit_cs.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void testBrakeUnitCSRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    brakeUnitCSInit();

    int32_t     counter = 0;
    int32_t     brakePower = 0;
    while ( 1 )
    {
        bool      isBrakePressed  = brakeSensorIsPressed();
        int16_t   pressPower      = brakeSensorGetPressPower();
        int16_t   pwmValue        = brakeUnitCSGetControl();
        int16_t   brakeVoltage    = brakeSensorGetVoltage();

        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {

         case 'z':   // Positive brake
             brakePower += 2;
             break;

         case 'x':   // Negative brake
             brakePower -= 2;
             break;

         default:
             ;
        }
        brakePower = CLIP_VALUE( brakePower, -2, 25 );
        brakeUnitCSSetPower( brakePower );
#if 1
        counter++;
        if ( counter >= 10 )
        {
          chprintf( (BaseSequentialStream *)&SD7, "Brake: %spressed, power: %d, ref: %d, pwm: %d, mV: %d\n\r",
                          isBrakePressed ? "" : "not ", pressPower, brakePower, pwmValue, brakeVoltage );

          counter = 0;
        }
#endif
        chThdSleepMilliseconds( 10 );
    }
}

void testBrakeUintOpenedRoutine( void )
{
     sdStart( &SD7, &sdcfg );
     palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
     palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

     brakeUnitCSInit();
     int32_t brakePower = 0;
     while( 1 )
     {
       bool    isBrakePressed  = brakeSensorIsPressed();
       int16_t pressPower      = brakeSensorGetPressPower();
       int16_t brakeVoltage    = brakeSensorGetVoltage();

       chprintf( (BaseSequentialStream *)&SD7, "Brake: %spressed, power: %d, mV: %d, brP: %d\n\r",
                       isBrakePressed ? "" : "not ", pressPower, brakeVoltage, brakePower );

       char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
       switch ( rcv_data )
       {

         case 'z':   // Positive brake
             brakePower += 10;
             brakePower = CLIP_VALUE( brakePower, -100, 100 );
             break;

         case 'x':   // Negative brake
             brakePower -= 10;
             brakePower = CLIP_VALUE( brakePower, -100, 100 );
             break;

         default:
             ;
       }

       lldControlSetBrakePower( brakePower );

       chThdSleepMilliseconds( 100 );

     }
}
