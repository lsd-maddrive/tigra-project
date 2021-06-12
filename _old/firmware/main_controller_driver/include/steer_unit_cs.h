
#ifndef INCLUDE_STEER_UNIT_CS_H_
#define INCLUDE_STEER_UNIT_CS_H_


#include <lld_steer_sensors.h>
#include <lld_control.h>

#include <light_unit.h>

#include <controllers.h>

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitCSInit( void );

/*
 * @brief   PID implementation of steering control system
 * @param   position   	Reference position [-100 100]
 * @return	Control value that is set to steer drive
 */
int32_t steerUnitCSSetPosition( int32_t position );

/**
 * @brief   Check ESC condition
 * @return  true  - ESC enable, everything - OK
 *          false - ESC is disabled or another bad situation
 * @note    WARNING! There is a delay (1 s) inside the function
 * @note    do not call it inside the loop, use it before Control System processing
 * @note    this function works only for determined phase order, be careful
 */
bool steerIsEnabled( void );

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitInit( void );


#endif /* INCLUDE_STEER_UNIT_CS_H_ */
