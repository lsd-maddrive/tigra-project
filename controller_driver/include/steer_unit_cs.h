
#ifndef INCLUDE_STEER_UNIT_CS_H_
#define INCLUDE_STEER_UNIT_CS_H_


#include <lld_steer_sensors.h>
#include <lld_control.h>

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



#endif /* INCLUDE_STEER_UNIT_CS_H_ */
