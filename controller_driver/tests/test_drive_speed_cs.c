#include <tests.h>
#include <drive_speed_cs.h>

int32_t speedReference  = 0;
uint8_t setMotorPower   = 0;

#if 1
static THD_WORKING_AREA(waDriveSpeedCSThd, 128);
static THD_FUNCTION(DriveSpeedCSThd, arg)
{
    arg = arg;


    wheelVelocity_t speedReference =  0;

    while ( 1 )
    {
//        speedReference =  speedReferenceMaxVal*( sin (ticksCounter)+1 );
        setMotorPower = DriveSpeedControl ( speedReference );
//        ticksCounter ++;
        chThdSleepMilliseconds(1);

    }
}
#endif

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
  DriveSpeedCSInit();

  chThdCreateStatic(waDriveSpeedCSThd, sizeof( waDriveSpeedCSThd ), NORMALPRIO, DriveSpeedCSThd,NULL);

  sdStart( &SD7, &sdcfg );
  palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
  palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

  while ( 1 )
  {
    wheelVelocity_t currentSpeed    =   wheelPosSensorGetVelocity ();
    chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n %s %d\r\n" , "Ref:",
                     (int) (speedReference*1000), "Vel:", (int) (currentSpeed*1000),
                     "Out:", (int) (setMotorPower*1000) );
    chThdSleepMilliseconds(100);
  }
}

