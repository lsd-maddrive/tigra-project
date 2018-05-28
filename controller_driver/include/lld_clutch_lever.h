#ifndef INCLUDE_LLD_CLUTCH_LEVER_H_
#define INCLUDE_LLD_CLUTCH_LEVER_H_

#include <common.h>


/*** Variables ***/

typedef void (*clutchLeverCb_t)( void );

/*** Prototypes ***/

/**
 * @brief       Initialization of clutch lever driver
 * @param[in]   callback    Callback function that is called when clutch lever
 *                          is pressed (front)
 * @note        Callback function is called inside ISR context
 */
void clutchLeverInit ( clutchLeverCb_t callback );

#endif /* INCLUDE_LLD_CLUTCH_LEVER_H_ */
