#include <tests.h>
#include <lld_sonars.h>



/*** Variable for sonar   */
uint16_t brownSonarVal = 0;
uint8_t buf5Son[4];

static THD_WORKING_AREA(waGetSonarValU5Thd, 1024);
static THD_FUNCTION(GetSonarValU5Thd, arg)
{
    arg = arg;
    uint8_t firstR = 0;

    while( 1 )
    {

      brownSonarVal = getSonarValU5cm( firstR, buf5Son );

    }
}

uint8_t buf7Son[4];
uint16_t greenSonarVal = 0;

static THD_WORKING_AREA(waGetSonarValU7Thd, 1024);
static THD_FUNCTION(GetSonarValU7Thd, arg)
{
    arg = arg;
    uint8_t firstR = 0;
    while( 1 )
    {

      greenSonarVal = getSonarValU7cm( firstR, buf7Son );

    }
}


/*
 * @brief   Routine of sonar sensors testing
 * @brief   Printing values (uint16_t) into Terminal
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void )
{
    lldSonarsInit( );

    chThdCreateStatic( waGetSonarValU5Thd, sizeof(waGetSonarValU5Thd), NORMALPRIO, GetSonarValU5Thd, NULL ); // brown
    chThdCreateStatic( waGetSonarValU7Thd, sizeof(waGetSonarValU7Thd), NORMALPRIO, GetSonarValU7Thd, NULL ); // green

    while( true )
    {
      chprintf( (BaseSequentialStream *)&SD7, "Br: %d Bl: %d\n\r", brownSonarVal, greenSonarVal );

      chThdSleepMilliseconds( 10 );

    }

}
