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
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU4cm( void );

/*
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU7cm( void );

#endif /* INCLUDE_LLD_SONARS_H_ */
