/*
 * lld_sonars.h
 *
 *  Created on: 05 θώνÿ 2018 γ.
 *      Author: Elena
 */

#ifndef INCLUDE_LLD_SONARS_H_
#define INCLUDE_LLD_SONARS_H_


/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void );

#if 0
/*
 * @brief                   Get Adc value of sonar
 * @return                  ADC value [0, 4096]
 */
//uint16_t lldSonar7077AdcVal( uint8_t number );

/*
 * @brief                   Set duty of PWM9 = 20 mks to sync sensor processing
 */
//void lldSonarSync( void );

#endif

#endif /* INCLUDE_LLD_SONARS_H_ */
