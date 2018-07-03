#ifndef INCLUDE_LIGHT_UNIT_H_
#define INCLUDE_LIGHT_UNIT_H_

#include <common.h>

typedef enum
{
    LIGHTS_TURN_RIGHT,
    LIGHTS_TURN_LEFT,
    LIGHTS_TURN_OFF,

    LIGHTS_BRAKE_ON,
    LIGHTS_BRAKE_OFF

} light_state_t;

/**
 * @brief   Initialize periphery of lights unit
 */
void lightUnitInit( void );

/**
 * @brief   Set state of lights
 * @param	state	State from enum <light_state_t>
 */
void turnLightsSetState( light_state_t state );

#endif /* INCLUDE_LIGHT_UNIT_H_ */
