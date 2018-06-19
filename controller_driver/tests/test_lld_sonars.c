#include <tests.h>
#include <lld_sonars.h>



/*** Variable for sonar   */
//uint16_t brownSonarVal = 0;
//uint8_t buf5Son[4];


/*
 * @brief   Routine of sonar sensors testing
 * @brief   Printing values (uint16_t) into Terminal
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void )
{
    lldSonarsInit( );
    uint16_t blueSonar = 0, brownSonar = 0;
    while( true )
    {
      palToggleLine( LINE_LED2 );
      blueSonar  =  getSonarValU7cm();
      brownSonar =  getSonarValU4cm();

      chprintf( (BaseSequentialStream *)&SD7, "Br: %d Bl: %d\n\r", brownSonar, blueSonar );

      chThdSleepMilliseconds( 100 );

    }

}
