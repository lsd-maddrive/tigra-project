#include <tests.h>
#include <course_drive_speed_cs.h>
#include <lld_control.h>


#if 0
static THD_WORKING_AREA(waDriveSpeedCSThd, 128);
static THD_FUNCTION(DriveSpeedCSThd, arg)
{
    arg = arg;

    /* Required for direct control */
    palSetPadMode( GPIOF, 14, PAL_MODE_OUTPUT_PUSHPULL );

    while ( 1 )
    {
        palTogglePad( GPIOF, 14 );

        //chThdSleepMilliseconds(2);
        //chThdSleepSeconds(40);
        chThdSleepMicroseconds(250);

    }
}
#endif

void testCourseDriveSpeedCSRoutine( void )
{
  //chThdCreateStatic(waDriveSpeedCSThd, sizeof( waDriveSpeedCSThd ), NORMALPRIO, DriveSpeedCSThd,NULL);
  CourseDriveSpeedCSInit();
   /*
    * wheelPosSensorInit();
    * lldControlInit();
    */
}

