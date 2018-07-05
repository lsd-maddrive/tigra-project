#include <tests.h>
#include <drive_speed_cs.h>

#include <math.h>

static const SerialConfig sdcfg = {
    .speed = 115200,
    .cr1 = 0, .cr2 = 0, .cr3 = 0
};


/*
 * @brief   Routine of motor control system  testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testDriveSpeedCSRoutine( void )
{
    /* Low level drivers initialization required for motor control*/
    driveSpeedCSInit();

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    wheelVelocity_t speedRef            = 0;
    wheelVelocity_t currentSpeed        = 0;
    uint32_t        printCntr           = 0;
    int32_t         motorControlValue   = 0;

    while ( 1 )
    {
        if ( ++printCntr == 10 )
        {
            currentSpeed = wheelPosSensorGetVelocity ();

            double speedRefIntPart;
            double speedRefFrctPart = modf( speedRef, &speedRefIntPart );

            double currSpeedIntPart;
            double currSpeedFrctPart = modf( currentSpeed, &currSpeedIntPart );

            chprintf( (BaseSequentialStream *)&SD7, "Ref: %d.%03d\tVel: %d.%03d\tOut: %d\r\n" ,
                        (int)(speedRefIntPart), (int)(speedRefFrctPart * 1000),
                        (int)(currSpeedIntPart), (int)(currSpeedFrctPart * 1000),
                        motorControlValue );

            printCntr = 0;
        }


        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'z':   // Positive ref
                speedRef += 0.1;
                break;

            case 'x':   // Negative ref
                speedRef -= 0.1;
                break;

            default:
                ;
        }

        /* Limit RPM value */
        speedRef = speedRef < 0 ? 0 : speedRef > 10 ? 10 : speedRef;

        motorControlValue   = driveSpeedControl ( speedRef );

        chThdSleepMilliseconds( 10 );

    }
}

void testDriveSpeedOpenedRoutine( void )
{
    /* Low level drivers initialization required for motor control*/
    driveSpeedCSInit();

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    int32_t speedPower = 0;

    uint32_t    sdCntr = 0;

    while( 1 )
    {
        if ( sdCntr++ > 10 )
        {
            sdCntr = 0;
            chprintf( (BaseSequentialStream *)&SD7, 
                        "pow: %d / rotTime: %d [ms] / vel: %d / velAn: %d\n\r", 
                        speedPower,
                        (int)(wheelPosSensorGetRotTime() * 1000),
                        (int)wheelPosSensorGetVelocity(),
                        (int)wheelPosSensorGetVelocityADC() );
        }

        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'c':
                speedPower += 5;
                speedPower = CLIP_VALUE( speedPower, 0, 100 );
                break;

            case 'v':
                speedPower -= 5;
                speedPower = CLIP_VALUE( speedPower, 0, 100 );
                break;

            default:
                ;
        }

        lldControlSetDrMotorPower( speedPower );

        chThdSleepMilliseconds( 10 );

    }

}

