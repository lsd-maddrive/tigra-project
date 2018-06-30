
#ifndef INCLUDE_STEER_UNIT_CS_H_
#define INCLUDE_STEER_UNIT_CS_H_


#include <lld_steer_sensors.h>

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitCSInit( void );

/*
 * @brief   PID implementation
 * @param   steerPower   Reference value [-100 100]
 */
int32_t steerUnitCSSetPower( int16_t steerPower );



#endif /* INCLUDE_STEER_UNIT_CS_H_ */
