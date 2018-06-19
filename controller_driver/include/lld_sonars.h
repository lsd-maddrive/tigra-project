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

/*
 * @brief                   Receive values of sonar (in cm) through UART5
 * @arg                     firstR - first byte from sonar, if sonar works correctly firstR = 'R'
 * @arg                     buf - buffer name (size = 4 byte)
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU5cm( uint8_t firstR, uint8_t buf[4] );

/*
 * @brief                   Receive values of sonar (in cm) through UART5
 * @arg                     firstR - first byte from sonar, if sonar works correctly firstR = 'R'
 * @arg                     buf - buffer name (size = 4 byte)
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU7cm( uint8_t firstR, uint8_t buf[4] );

#endif /* INCLUDE_LLD_SONARS_H_ */
