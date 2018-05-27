#include <tests.h>
#include <course_drive_speed_cs.h>
#include <lld_control.h>
//#include <math.h>


#if 1
static THD_WORKING_AREA(waDriveSpeedCSThd, 128);
static THD_FUNCTION(DriveSpeedCSThd, arg)
{
    arg = arg;

    uint8_t setMotorPower = 0;
    wheelVelocity_t speedReference =  0;

    while ( 1 )
    {
        speedReference =  speedReferenceMaxVal*( sin (ticksCounter)+1 );
        setMotorPower = CourseDriveSpeedControl ( speedReference );
        ticksCounter ++;
        chThdSleepMilleseconds(1);

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
void testCourseDriveSpeedCSRoutine( void )
{
  /* Low level drivers initialization required for motor control*/
  CourseDriveSpeedCSInit();

  chThdCreateStatic(waDriveSpeedCSThd, sizeof( waDriveSpeedCSThd ), NORMALPRIO, DriveSpeedCSThd,NULL);

  sdStart( &SD7, &sdcfg );
  palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
  palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

  while (1)
  {
    wheelVelocity_t currentSpeed    =   wheelPosSensorGetVelocity ();
    chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n %s %d\r\n" , "Ref:",
                     (int) (speedReference*1000), "Vel:", (int) (currentSpeed*1000),
                     "Out:", (int) (setMotorPower*1000) );
    chThdSleepMilleseconds(100);
  }
}

