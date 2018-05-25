#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <hal.h>
#include <ch.h>

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
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER          2
#define     PROGRAM_ROUTINE_TEST_CLUTCH_LEVER       10

#define     MAIN_PROGRAM_ROUTINE                    PROGRAM_ROUTINE_TEST_LL_DRIVER


/*** Prototypes ***/

/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void );

#endif /* INCLUDE_COMMON_H_ */
