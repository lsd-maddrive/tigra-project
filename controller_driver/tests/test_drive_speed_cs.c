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

    int32_t         speedRef            = 0;
    wheelVelocity_t currentSpeed        = 0;
    uint32_t        printCntr           = 0;
    int32_t         motorControlValue   = 0;

    while ( 1 )
    {
        if ( ++printCntr == 10 )
        {
            currentSpeed = wheelPosSensorGetVelocity ();

            // double currSpeedIntPart;
            // double currSpeedFrctPart = modf( currentSpeed, &currSpeedIntPart );

            chprintf( (BaseSequentialStream *)&SD7, "Ref: %d\tVel: %d\tOut: %d\r\n" ,
                        speedRef, (int)currentSpeed,
                        // (int)(currSpeedIntPart), (int)(currSpeedFrctPart * 1000),
                        motorControlValue );

            printCntr = 0;
        }


        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'z':
                speedRef += 10;
                break;

            case 'x':
                speedRef -= 10;
                break;

            case ' ':
                speedRef = 0;
                break;

            case '1':
                speedRef = 50;
                break;

            case '2':
                speedRef = -50;
                break;


            default:
                ;
        }   
        speedRef = CLIP_VALUE( speedRef, -100, 100 );

        /* Limit RPM value */

        motorControlValue   = driveSpeedControl ( speedRef );
        // lldControlSetDrMotorPower( speedRef );

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

    int32_t speedPower  = 0;
    int32_t ctrlPower   = 0;

    uint32_t    sdCntr = 0;

    while( 1 )
    {
        if ( sdCntr++ > 10 )
        {
            sdCntr = 0;
            chprintf( (BaseSequentialStream *)&SD7, 
                        "pow: %d / rotTime: %d [ms] / vel: %d / spd: %d / ctrl: %d\n\r", 
                        speedPower,
                        (int)(wheelPosSensorGetRotTime() * 1000),
                        (int)wheelPosSensorGetVelocity(),
                        (int)wheelPosSensorGetLinSpeed(),
                        ctrlPower );
        }

        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'c':
                speedPower += 5;
                break;

            case 'v':
                speedPower -= 5;
                break;

            case '1':
                speedPower = 50;
                break;

            case '2':
                speedPower = -50;
                break;

            case ' ':
                speedPower = 0;
                break;

            default:
                ;
        }

        speedPower = CLIP_VALUE( speedPower, -100, 100 );
        // ctrlPower = lldControlSetDrMotorPower( speedPower );

        /*** Bad way -- TODO - recover ***/
        ctrlPower = driveSpeedControl ( speedPower );

        chThdSleepMilliseconds( 10 );

    }

}

