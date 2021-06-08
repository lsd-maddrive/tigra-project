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

    int32_t     refPosition = 0;

    while ( 1 )
    {
        /*
         *  It's imitation of 5s delay to let ESC now where the center is
         *  Actually, delay could be reduced (5s is too much)
         *  it should work only once, when we turn on ESC first time
         */
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

            int32_t currentPosSensor        = lldSteerGetPositionPrc();
            int32_t control                 = steerUnitCSSetPosition( refPosition );
            int32_t posADC                  = lldSteerGetPositionADC(); 


            if ( ++counterT >= 10 )
            {
                chprintf( (BaseSequentialStream *)&SD7, 
                                "Ref: %d, Control: %d, posSensor: %d, adc: %d\n\r", 
                                refPosition, control, currentPosSensor, posADC );

                counterT = 0;
            }

            char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
            switch ( rcv_data )
            {
                case 'd':   // Positive brake
                    refPosition += 10;
                    break;

                case 'f':   // Negative brake
                    refPosition -= 10;
                    break;

                default:
                  ;
            }

            refPosition = CLIP_VALUE( refPosition, -100, 100 );
        }

        chThdSleepMilliseconds( 10 );
    }


}
