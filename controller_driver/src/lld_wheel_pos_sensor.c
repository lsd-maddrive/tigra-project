#include <lld_wheel_pos_sensor.h>


/* Wheel position sensor connects to wheelPosSensorInLine pin*/
#define wheelPosSensorInLine       PAL_LINE ( GPIOF, 13 )

/* PartOfWheelRevPerMinute stands for part of revolution per minute
 * uses for velocity calculation   */
const  float PartOfWheelRevPerMinute =    (float) ( 60 / ImpsPerRevQuantity ) ;

static void extcb ( EXTDriver *extp, expchannel_t channel );

static void gpt_overflow_cb ( GPTDriver *timeIntervalsDriver );
static GPTDriver                        *timeIntervalsDriver = &GPTD3;
/* Timer period */
#define TimerPeriod             50000
static const GPTConfig timeIntervalsCfg = {
                                             .frequency      =  1000000,// 1 MHz
                                             .callback       =  gpt_overflow_cb,
                                             .cr2            =  0,
                                             .dier           =  0U

};

uint32_t impulseCounter = 0,  prev_time = 0, measured_width = 0;
uint32_t overflow_counter = 0;
static bool         isInitialized       = false;

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void wheelPosSensorInit (void)
{

    /* Define channel config structure */
    EXTChannelConfig ch_conf;

    /* Fill in configuration for channel */
    ch_conf.mode  = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOF;
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

    /* Start working serial driver */
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    isInitialized       = true;

}

/* Timer 3 overflow callback function */
static void gpt_overflow_cb(GPTDriver *gptd)
{
    gptd = gptd;

    /* Increment overflow counter*/
    overflow_counter ++;
}


/* Callback function of the EXT
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
}

/**
 * @ brief                            Gets wheel current velocity value
 *                                    [revolutions per minute (rpm)]
 * @ param[in] ImpsPerRevQuantity     Number of impulses per revolution
 *                                    depends on given sensor
 * @ return                           Current wheel velocity value [rpm]
 *
 */
wheelVelocity_t wheelPosSensorGetVelocity ( void )
{
    wheelVelocity_t  velocity = 0;
    
    if ( !isInitialized )
    {
        return -1;
    }

    /* Protection of devision by zero.
     * measured_width = 0 if fronts counter < 2,
     * which means start and probably incorrect velocity calculation */
    if ( measured_width != 0)
    {
        velocity = (float) ( PartOfWheelRevPerMinute * timeIntervalsCfg.frequency )/ ( measured_width );
    }
    else
    {
        velocity = 0;
    }

    return velocity;
}

/**
 * @ brief                    Sends current timer counter value to serial
 */
void sendTestInformation (void)
{
  wheelVelocity_t vel = wheelPosSensorGetVelocity ();
  vel = vel*100;
  chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n %s %d\r\n %s %d\r\n" ,
              "prev time:", prev_time, "time width (tick):", measured_width,
              "velocity", (int)vel, "ovflow_ctr", overflow_counter );
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
    position = impulseCounter/ImpsPerRevQuantity;
    return position;
}
