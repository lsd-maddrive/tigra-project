#include <tests.h>
#include <chprintf.h>
#include <lld_break_sensor.h>

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BREAK_SENSOR)
static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/*
 * @brief	Routine of break sensor testing
 * @note	The routine has internal infinite loop
 * @note 	SD7 is used for testing (PE7, PE8)
 */
void testBreakSensorRoutine( void )
{
/* Cover into #if to avoid code size growth */
#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_BREAK_SENSOR)

	sdStart( &SD7, &sdcfg );
	palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );	// TX
	palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );	// RX

	breakSensorInit();

	while ( 1 )
	{
		bool				isBreakPressed  = breakSensorIsPressed();
		breakPressPower_t	pressPower		= breakSensorGetPressPower();

		chprintf( (BaseSequentialStream *)&SD7, "Break: %spressed, power: %d",
						isBreakPressed ? "" : "not ", pressPower );

		chThdSleepMilliseconds( 100 );
	}

#endif
}
