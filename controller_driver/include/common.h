#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <hal.h>
#include <ch.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/*********************/
/*** Configuration ***/
/*********************/

/* Program routine selection */
/*
 * Bad idea to make it <enum> as we use #if (preprocessor) to
 * check what routine to use
 * <enum> is compilation type, not preprocessor
 * This must be #define
 */
/* !! When you define new variable use much higher values than last defined !! */
#define     PROGRAM_ROUTINE_MASTER                  0

#define     PROGRAM_ROUTINE_TEST_BRAKE_SENSOR       1
#define     PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR   2
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER          3
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER_EXT1     4
#define     PROGRAM_ROUTINE_TEST_CLUTCH_LEVER       5
#define     PROGRAM_ROUTINE_TEST_BRAKE_UNIT_CS      6
#define     PROGRAM_ROUTINE_TEST_BRAKE_UNIT_OPENED  7
#define     PROGRAM_ROUTINE_TEST_STEER_SENSORS      8
#define     PROGRAM_ROUTINE_TEST_ROS_DRIVER         9
#define     PROGRAM_ROUTINE_TEST_LL_SONAR           10
#define     PROGRAM_ROUTINE_TEST_LL_SHARP           11



#define     MAIN_PROGRAM_ROUTINE                    PROGRAM_ROUTINE_TEST_BRAKE_SENSOR

/******************/
/*** Prototypes ***/
/******************/

/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void );

/*** Common ADC configuration ***/

#define COMMON_ADC_RES_CONF         ADC_CR1_12B_RESOLUTION
#define COMMON_ADC_BUFFER_DEPTH     4

/* Brake sensor current input */
#define COMMON_ADC_SEQ1             ADC_CHANNEL_IN3
#define COMMON_ADC_SEQ1_LINE        PAL_LINE( GPIOA, 3 )
#define COMMON_ADC_SEQ1_CH          0

/* Steer sensor position input */
#define COMMON_ADC_SEQ2             ADC_CHANNEL_IN10
#define COMMON_ADC_SEQ2_LINE        PAL_LINE( GPIOC, 0 )
#define COMMON_ADC_SEQ2_CH          1

/* Steer sensor current input */
#define COMMON_ADC_SEQ3             ADC_CHANNEL_IN13
#define COMMON_ADC_SEQ3_LINE        PAL_LINE( GPIOC, 3 )
#define COMMON_ADC_SEQ3_CH          2


/**
 * @brief   Initialize common ADC1 unit
 * @note    Safe to call any times, it checks state of previous call
 */
void commonADC1UnitInit ( void );

/**
 * @brief       Get value of desired channel
 * @param   ch  Number of channel
 * @return      Filtered (if realized) value of ADC sampling
 *              Zero if channel is invalid 
 */
adcsample_t commonADC1UnitGetValue ( uint8_t ch );

/**************/
/*** Macros ***/
/**************/

#define CLIP_VALUE(x, min, max) ((x) < (min) ? (min) :      \
                                 (x) > (max) ? (max) : (x))

/*****************/
/*** Constants ***/
/*****************/

/* Sensor ref: https://www.sparkfun.com/datasheets/BreakoutBoards/0712.pdf */
/* Rates for ACS712 sensors [mv/A] */
#define ACS712_5AMP_RATE        185.0
#define ACS712_20AMP_RATE       100.0
#define ACS712_30AMP_RATE       66.0


/* Additional ADC constants */
#define ADC_CR1_12B_RESOLUTION      (0)
#define ADC_CR1_10B_RESOLUTION      (ADC_CR1_RES_0)
#define ADC_CR1_8B_RESOLUTION       (ADC_CR1_RES_1)
#define ADC_CR1_6B_RESOLUTION       (ADC_CR1_RES_0 | ADC_CR1_RES_1)


#endif /* INCLUDE_COMMON_H_ */
