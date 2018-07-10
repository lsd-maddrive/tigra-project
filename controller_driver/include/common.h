#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <hal.h>
#include <ch.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>

/* For errors signaling */
#ifndef EOK
    #define EOK     0
#endif

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

#define     PROGRAM_ROUTINE_MASTER                      0

#define     PROGRAM_ROUTINE_TEST_BRAKE_SENSOR           1
#define     PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR       2
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER              3
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER_SERIAL       4
#define     PROGRAM_ROUTINE_TEST_CLUTCH_LEVER           5
#define     PROGRAM_ROUTINE_TEST_BRAKE_UNIT_CS          6
#define     PROGRAM_ROUTINE_TEST_BRAKE_UNIT_OPENED      7
#define     PROGRAM_ROUTINE_TEST_DRIVE_SPEED_CS         8
#define     PROGRAM_ROUTINE_TEST_DRIVE_SPEED_OPENED     9
#define     PROGRAM_ROUTINE_TEST_STEER_UNIT_CS          10
#define     PROGRAM_ROUTINE_TEST_STEER_SENSORS          11
#define     PROGRAM_ROUTINE_TEST_STEER_POWERED          12
#define     PROGRAM_ROUTINE_TEST_ROS_DRIVER             13
#define     PROGRAM_ROUTINE_TEST_LL_SONAR               14
#define     PROGRAM_ROUTINE_TEST_LL_SHARP               15
#define     PROGRAM_ROUTINE_TEST_TOOLS_MATLAB_SLIDER    16
#define     PROGRAM_ROUTINE_TEST_LIGHTNING              17
#define     PROGRAM_ROUTINE_TEST_LL_DRIVER_STEER        18

#define     MAIN_PROGRAM_ROUTINE                        PROGRAM_ROUTINE_TEST_WHEEL_POS_SENSOR

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

/*      Speed sensor input    */
#define COMMON_ADC_SEQ4             ADC_CHANNEL_IN12
#define COMMON_ADC_SEQ4_LINE        PAL_LINE( GPIOC, 2 )
#define COMMON_ADC_SEQ4_CH          3


typedef float   adc1SampleMV_t;

/**
 * @brief   Initialize common ADC1 unit
 * @note    Safe to call any times, it checks state of previous call
 */
void commonADC1UnitInit ( void );

/**
 * @brief       Get value of required channel
 * @param   ch  Number of channel
 * @return      Filtered (if realized) value of ADC sampling
 *              Zero if channel is invalid 
 */
adcsample_t commonADC1UnitGetValue ( uint8_t ch );

/**
 * @brief       Get value of required channel in units
 * @param ch    Number of channel
 * @return      Filtered (if realized) value of ADC sampling [mV]
 *              Zero if channel is invalid 
 */
adc1SampleMV_t commonADC1UnitGetValueMV ( uint8_t ch );

/*** Main control task protos ***/

/**
 * @brief       Main control loop
 * @note        Has internal loop inside
 */
void mainControlTask ( void );

/**
 * @brief       Main periphery initialization
 * @return      Error code:
 *              EOK     - everything initialized
 *              EIO     - steering test failed
 * @note        Must be called first before main_control_task()
 */
int mainUnitsInit ( void );

/**
 * @brief           Set task from external control
 * @param   speed   Desired speed of base
 * @param   steer   Desired steer angle
 */
void mainControlSetTask ( int32_t speed, int32_t steer );

typedef struct
{
    int32_t speedTask;
    int32_t steerTask;

    uint8_t mode;

} mainControlInfo_t;

/**
 * @brief   Debug function
 * @return  Structure with main info from controller
 */
mainControlInfo_t mainControlGetInfo( void );

/**
 * @brief       set working mode
 */
void mainControlSetMode( uint8_t mode );


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
