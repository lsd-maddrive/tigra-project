#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <hal.h>
#include <ch.h>

/*** Configuration ***/

/* Program routine selection */
typedef enum {
	PROGRAM_ROUTINE_MASTER,
	PROGRAM_ROUTINE_TEST_BREAK_SENSOR

} program_routine_t;

#define MAIN_PROGRAM_ROUTINE				PROGRAM_ROUTINE_MASTER

/*** Prototypes ***/

#endif /* INCLUDE_COMMON_H_ */
