/*
 * lld_sonars.h
 *
 *  Created on: 05 θώνÿ 2018 γ.
 *      Author: Elena
 */

#ifndef INCLUDE_LLD_SONARS_H_
#define INCLUDE_LLD_SONARS_H_



#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SHARP)
/*
 * @brief                   Initialize periphery connected to IR-sensor
 */
void lldSharpInit( void );

/*
 * @brief                   Return ADC value from IR-sensor (refreshed val - 10 Hz)
 */
uint16_t lldSharpADCval( void );

#endif


#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SONAR)
/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void );

/*
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU4cm( void );

/*
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU5cm( void );

#endif
#endif /* INCLUDE_LLD_SONARS_H_ */
