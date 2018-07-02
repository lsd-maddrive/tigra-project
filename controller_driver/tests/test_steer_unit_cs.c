#include <tests.h>
#include <chprintf.h>
#include <steer_unit_cs.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

int32_t     counterT = 0, startCount = 0, flag = 0;

void testSteerUnitCSRoutine( void )
{

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    steerUnitCSInit();



    int32_t     steerPower = 2065;

    while ( 1 )
    {
        if( flag == 0 )
        {
            startCount += 1;
        }

        if( startCount <= 500 )
        {
          lldControlSetSteerPower( 0 );
          chprintf( (BaseSequentialStream *)&SD7, "WAIT\n\r" );

        }
        else if( startCount > 500 )
        {
          flag = 1;
        }

        if( flag == 1)
        {

          int32_t currentPosSensor = lldSteerGetPosition();
          int32_t control      = steerUnitCSSetPower( steerPower );




          if ( ++counterT >= 10 )
          {
              chprintf( (BaseSequentialStream *)&SD7, "Ref: %d, Control: %d, posSensor: %d\n\r", steerPower, control, currentPosSensor );

              counterT = 0;
          }

          char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
          switch ( rcv_data )
          {
              case 'd':   // Positive brake
                  steerPower += 100;
                  break;

              case 'f':   // Negative brake
                  steerPower -= 100;
                  break;

              default:
                  ;
          }

          steerPower = CLIP_VALUE( steerPower, 1190, 2800 );
        }
        chThdSleepMilliseconds( 10 );
    }


}
