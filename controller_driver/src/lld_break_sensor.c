#include <lld_break_sensor.h>

/*
 * @brief 	Initialize periphery connected to break sensor
 */
void breakSensorInit ( void )
{

}

/*
 * @brief	Check if break is pressed
 * @return 	true  - break is pressed
 * 			false - break is not pressed
 */
bool breakSensorIsPressed ( void )
{

}

/*
 * @brief	Get press power value
 * @return	[0, 100] - Press power percentage
 * 			< 0 	 - Sensor not initialized
 */
breakPressPower_t breakSensorGetPressPower ( void )
{
	breakPressPower_t value = 0;

	if ( breakSensorIsPressed() )
	{

	}

	return value;
}
