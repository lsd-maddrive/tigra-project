#include <lld_wheel_pos_sensor.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

const uint32_t  wheelSpeedSensorTimeoutMs       = 1000;

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/* Wheel position sensor connects to wheelPosSensorInLine pin*/
#define wheelPosSensorInLine       PAL_LINE ( GPIOF, 13 )


static void extcb ( EXTDriver *extp, expchannel_t channel );

static void gpt_overflow_cb ( GPTDriver *timeIntervalsDriver );
static GPTDriver                        *timeIntervalsDriver = &GPTD3;
/* Timer period = 50 ms */
#define TimerPeriod             50000
static const GPTConfig timeIntervalsCfg = {
                                             .frequency      =  1000000,// 1 MHz
                                             .callback       =  gpt_overflow_cb,
                                             .cr2            =  0,
                                             .dier           =  0U

};

int32_t impulseCounter = 0,  prev_time = 0, measured_width = 0;
int32_t overflow_counter = 0;
static bool         isInitialized                   = false;
static bool         wheelsRotating                  = false;

static int32_t      wheelSpeedSensorMaxOverflows    = 0;

/* PartOfWheelRevPerMinute stands for part of revolution per minute
 * uses for velocity calculation   */
static float        velocityCalcTicksToRPM  = 0;

void wheelPosSensorInit (void)
{
    /* Define channel config structure */
    EXTChannelConfig ch_conf;

    /* Fill in configuration for channel */
//    ch_conf.mode  = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOF;
    ch_conf.mode  = EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOF;

    ch_conf.cb    = extcb;

    /*EXT driver initialization*/
    commonExtDriverInit();

    /* Set channel (second arg) mode with filled configuration */
    extSetChannelMode( &EXTD1, 13, &ch_conf );

    /* Set up EXT channel hardware pin mode as digital input  */
    palSetLineMode( wheelPosSensorInLine, PAL_MODE_INPUT_PULLUP );

    /* Start working GPT driver in continuous (asynchronous) mode */
    gptStart( timeIntervalsDriver, &timeIntervalsCfg );
    gptStartContinuous( timeIntervalsDriver, TimerPeriod );

    isInitialized       = true;

    /* Some calculations */
    wheelSpeedSensorMaxOverflows = wheelSpeedSensorTimeoutMs / 
                                        (TimerPeriod * 1000.0 /* to ms */ / timeIntervalsCfg.frequency);

    velocityCalcTicksToRPM  = 60.0 * timeIntervalsCfg.frequency / ImpsPerRevQuantity;
}

/* Timer 3 overflow callback function */
static void gpt_overflow_cb(GPTDriver *gptd)
{
    gptd = gptd;

    /* Increment overflow counter*/
    overflow_counter ++;

    if ( overflow_counter >= wheelSpeedSensorMaxOverflows )
    {
        wheelsRotating = false;
    }
}

#define NEW_ALGORITHM

/**
 * Callback function of the EXT
 * It is triggered on event that is configured in config structure
 * args:  <extp>    - pointer to the driver, now it has
 *                    the only driver (EXTD1) and pointer will be like &EXTD1
 *        <channel> - channel where callback was triggered,
 *                    if trigger was from PD_0 - channel equals 0,
 *                    if from PB_3 - channel equals 3, like index in array
 */
static void extcb(EXTDriver *extp, expchannel_t channel)
{
    /* The input arguments are not used now */
    /* Just to avoid Warning from compiler */
    extp = extp; channel = channel;

#ifdef NEW_ALGORITHM

    int32_t curr_time   = gptGetCounterX(timeIntervalsDriver);
    measured_width      = 0;

    if ( wheelsRotating )
    {
        measured_width = curr_time + (overflow_counter-1) * TimerPeriod + (TimerPeriod - prev_time) ;
    }

    wheelsRotating = true;

    overflow_counter = 0;
    prev_time = curr_time;
    impulseCounter++;

#else
    /* Increment fronts counter  */
    impulseCounter ++;
    /* Calculate time (tics) width between two fronts */
    measured_width = gptGetCounterX(timeIntervalsDriver)+ (overflow_counter-1)*TimerPeriod + (TimerPeriod - prev_time) ;
    /* Save current timer counter value for next width calculation*/
    prev_time =  gptGetCounterX(timeIntervalsDriver);
    /* Reset timer overflows counter*/
    overflow_counter = 0;
    /* Protection of incorrect velocity calculation */
    /* Because measured_width calculations depends on  prev_time value  */
    /* If impulseCounter < 2 then prev_time =  0 */
    if ( impulseCounter < 2)
    {
        measured_width = 0;
    }
#endif
}

/**
 * @ brief                            Gets wheel current velocity value
 *                                    [revolutions per minute (rpm)]
 * @ param[in] ImpsPerRevQuantity     Number of impulses per revolution
 *                                    depends on given sensor
 * @ return                           Current wheel velocity value [rpm]
 */
wheelVelocity_t wheelPosSensorGetVelocity ( void )
{
    wheelVelocity_t  velocity = 0;
    
    if ( !isInitialized )
    {
        return -1;
    }

#ifdef NEW_ALGORITHM
    
    if ( !wheelsRotating )
        return 0;

#endif

    /* Protection of division by zero.
     * measured_width = 0 if fronts counter < 2,
     * which means start and probably incorrect velocity calculation */
    if ( measured_width != 0)
    {
        /**
         * second between fronts (Tf) = measured_width / frequency
         * there are 4 (config) fronts per revolution ~ 4 x Tf = second for revolution (Tr)
         * rps = 1 / Tr = 1 / (4 x Tf) = freq / (4 x ticks)
         * rpm = 60 * rps = 60 * freq / (4 x ticks)
         */
        velocity = velocityCalcTicksToRPM / measured_width;
    }
    else
    {
        velocity = 0;
    }

    return velocity;
}

/**
 * @ brief                           Gets wheel current position value
 *                                   [revolutions]
 * @ param[in] ImpsPerRevQuantity    Number of impulses per revolution
 *                                   depends on given sensor
 * @ return                          Current wheel position value [revolutions]
 *
 */
wheelPosition_t wheelPosSensorGetPosition ( void )
{
    wheelPosition_t position = 0;
    
    if ( !isInitialized )
    {
      return -1;
    }

    position = impulseCounter * 1.0 / ImpsPerRevQuantity;

    return position;
}
