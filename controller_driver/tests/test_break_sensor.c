#include <tests.h>
#include <chprintf.h>
#include <lld_break_sensor.h>

#ifdef TEST_BREAK_SENSOR_SIMULATED

#include <math.h>

static const DACConfig dac_cfg = {
	.init         = 0,
	.datamode     = DAC_DHRM_12BIT_RIGHT,
	.cr           = 0
};

static THD_WORKING_AREA(waDACThd, 128);
static THD_FUNCTION(DACThd, arg) 
{
	arg = arg;

	palSetPadMode( GPIOA, 4, PAL_MODE_INPUT_ANALOG );
	
	uint32_t	tick_count = 0;

	while ( 1 )
	{
		uint16_t value = ((1 << 11) - 1) * (sin( tick_count / 100.0 ) + 1);

		dacPutChannelX( &DACD1, 0, value );

		tick_count++;

		chThdSleepMilliseconds( 50 );
	}
}

static THD_WORKING_AREA(waBtnThd, 128);
static THD_FUNCTION(BtnThd, arg) 
{
	arg = arg;

	/* Required for direct control */
	palSetPadMode( GPIOA, 0, PAL_MODE_OUTPUT_PUSHPULL );

	while ( 1 )
	{
		palTogglePad( GPIOA, 0 );
		chThdSleepSeconds( 1 );
	}
}

static void simulation_init ( void )
{
	dacStart(&DACD1, &dac_cfg);

	chThdCreateStatic( waDACThd, sizeof(waDACThd), NORMALPRIO, DACThd, NULL );
	chThdCreateStatic( waBtnThd, sizeof(waBtnThd), NORMALPRIO, BtnThd, NULL );
}

#endif

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

/*
 * @brief	Routine of break sensor testing
 * @note	The routine has internal infinite loop
 * @note 	SD7 is used for testing (PE7, PE8)
 */
void testBreakSensorRoutine( void )
{
#ifdef TEST_BREAK_SENSOR_SIMULATED
	simulation_init();
#endif

	sdStart( &SD7, &sdcfg );
	palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );	// TX
	palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );	// RX

	breakSensorInit();

	while ( 1 )
	{
		bool				isBreakPressed  = breakSensorIsPressed();
		breakPressPower_t	pressPower		= breakSensorGetPressPower();

		chprintf( (BaseSequentialStream *)&SD7, "Break: %spressed, power: %d\n",
						isBreakPressed ? "" : "not ", pressPower );

		chThdSleepMilliseconds( 100 );
	}
}
