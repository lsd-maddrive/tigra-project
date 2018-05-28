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
/* !! When you define new variable user much higher values than last defined !! */
#define     PROGRAM_ROUTINE_MASTER                  0

#define     PROGRAM_ROUTINE_TEST_BRAKE_SENSOR       1
#define     PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR   2
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER          3
#define     PROGRAM_ROUTINE_TEST_CLUTCH_LEVER       4
#define     PROGRAM_ROUTINE_TEST_BRAKE_UNIT_CS      33

#define     MAIN_PROGRAM_ROUTINE                    PROGRAM_ROUTINE_MASTER

/*** Prototypes ***/

/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void );

/*** Macros ***/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))

#endif /* INCLUDE_COMMON_H_ */
