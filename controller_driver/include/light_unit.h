
#ifndef INCLUDE_LIGHT_UNIT_H_
#define INCLUDE_LIGHT_UNIT_H_

#include <common.h>

typedef enum
{
    LIGHTS_TURN_RIGHT,
    LIGHTS_TURN_LEFT,
    LIGHTS_OFF
} light_states_t;

/**
 * @brief   Initialize periphery connected to driver control
 */
void lightUnitInit( void );

/**
 * @brief   Set state ONLY for turn light unit depend on control signal value
 */
void turnLightsSetState( int32_t controlSignal );

#endif /* INCLUDE_LIGHT_UNIT_H_ */
