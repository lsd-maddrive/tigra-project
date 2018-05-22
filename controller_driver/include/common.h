#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <hal.h>
#include <ch.h>

/* Header for errors */
#include <errno.h>
/* Sometimes this error code is not defined */
#ifndef EOK
	#define EOK 	0
#endif

/*** Configuration ***/

/* Program routine selection */
/*
 * Bad idea to make it <enum> as we use #if (preprocessor) to
 * check what routine to use
 * <enum> is compilation type, not preprocessor
 * This must be #define
 */
#define     PROGRAM_ROUTINE_MASTER                  0
#define     PROGRAM_ROUTINE_TEST_BREAK_SENSOR       1
#define     PROGRAM_ROUTINE_TEST_CLUTCH_LEVER       10
#define     PROGRAM_ROUTINE_TEST_BLACK_BOX       	11

#define     MAIN_PROGRAM_ROUTINE                    PROGRAM_ROUTINE_TEST_BLACK_BOX

/*** Prototypes ***/

/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void );

#endif /* INCLUDE_COMMON_H_ */
