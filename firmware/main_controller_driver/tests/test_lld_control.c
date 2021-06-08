#include <tests.h>
#include <lld_control.h>
#include <chprintf.h>

void testDriverControlRoutine( void )
{
    lldControlInit( );

    lldControlSetSteerPower( 50 );
    palSetLine( LINE_LED1 );
    lldControlSetBrakePower( 75 );
    lldControlSetDrMotorPower( 0 );
    int8_t test_val = 75;
    while( true )
    {

       lldControlSetBrakePower( test_val );
       test_val *= -1;

       chThdSleepMilliseconds(500);
    }
}

/***************************************************/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testDriverControlRoutineSerial( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldControlInit();


    controlValue_t  steer_values_delta  = 10;
    controlValue_t  steer_value         = 0;

    controlValue_t  speed_values_delta  = 5;
    controlValue_t  speed_value         = 0;

    controlValue_t  brake_values_delta  = 10;
    controlValue_t  brake_value         = 0;

    chprintf( (BaseSequentialStream *)&SD7, "TEST\n\r" );

    while ( 1 )
    {
        char rcv_data = sdGet( &SD7 );
        switch ( rcv_data )
        {
            case 'q':   // Positive steer
                steer_value += steer_values_delta;
                break;

            case 'w':   // Negative steer
                steer_value -= steer_values_delta;
                break;

            case 'a':   // Positive speed
                speed_value += speed_values_delta;
                break;

            case 's':   // Negative speed
                speed_value -= speed_values_delta;
                break;

            case 'z':   // Positive brake
                brake_value += brake_values_delta;
                break;

            case 'x':   // Negative brake
                brake_value -= brake_values_delta;
                break;
            default:
                ;
        }

        steer_value = CLIP_VALUE( steer_value, -100, 100 );
        speed_value = CLIP_VALUE( speed_value, -100, 100 );
        brake_value = CLIP_VALUE( brake_value, -100, 100 );

        lldControlSetSteerPower( steer_value );

//        lldControlSetDrMotorDirection( steer_value > 0 );
        lldControlSetDrMotorPower( speed_value );

        lldControlSetBrakePower( brake_value );

        chprintf( (BaseSequentialStream *)&SD7, "Powers:\n\r\tSteer(%d)\n\r\tSpeed(%d)\n\r\tBrake(%d)\n\r",
                  steer_value, speed_value, brake_value );
        chThdSleepMilliseconds( 100 );
    }
}


void testDriverControlSteerRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldControlInit();
    lldSteerSensorsInit();
    
    controlValue_t  steer_values_delta  = 10;
    controlValue_t  steer_value         = 0;

    int32_t         printCntr           = 0;

    while ( 1 )
    {
        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'q':
                steer_value += steer_values_delta;
                break;

            case 'w':
                steer_value -= steer_values_delta;
                break;

            case '1':
                steer_value = 50;
                break;

            case '2':
                steer_value = -50;
                break;

            case '3':
                steer_value = 0;
                break;

            case '9':
                steer_value = 100;
                break;

            case '0':
                steer_value = -100;
                break;

            default:
                ;
        }

        steer_value = CLIP_VALUE( steer_value, -100, 100 );

        lldControlSetSteerPower( steer_value );

        if ( ++printCntr > 10 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Steer (%d) / ADC(%d)\n", steer_value, lldSteerGetPositionADC() );

            printCntr = 0;
        }
        
        chThdSleepMilliseconds( 10 );
    }
}
