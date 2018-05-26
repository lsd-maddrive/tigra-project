#include <lld_wheel_pos_sensor.h>
#include <chprintf.h>

#define wheelPosSensorInLine       PAL_LINE(GPIOF, 13)
const   uint32_t PartOfWheelRevPerMinute =    60 / ImpsPerRevQuantity;

static void extcb(EXTDriver *extp, expchannel_t channel);
static EXTConfig extcfg;
uint32_t impulseCounter = 0;
static void gpt_overflow_cb(GPTDriver *timeIntervalsDriver);
static GPTDriver                        *timeIntervalsDriver = &GPTD3;
static const GPTConfig timeIntervalsCfg = {
                                             .frequency      =  1000000,// 1 MHz
                                             .callback       =  gpt_overflow_cb,
                                             //.callback       =  NULL,
                                             //.cr2            =  TIM_CR2_MMS_1,
                                             .cr2            =  0,
                                             .dier           =  0U

};
/*
static void icu_width_cb(ICUDriver *icup);
static ICUDriver                     *icuDriver =   &ICUD1;
static const ICUConfig icuCfg = {
                                  .frequency    = 1000000,
                                  .mode         = ICU_INPUT_ACTIVE_HIGH,
                                  .overflow_cb  = NULL,
                                  .period_cb    = NULL,
                                  .width_cb     = icu_width_cb,
                                  /*
                                   * Select channel to measure pulse on
                                   * The issue of STM32 - ICU module captures full timer for only one pulse measurement
                                   * <.channel> selects first (ICU_CHANNEL_1) or second (ICU_CHANNEL_2) channel as main input
                                   * Only first two channels can be used
                                   * Respectively, ICU_CHANNEL_1 - connect pulse to timer channel 1

                                  .channel        = ICU_CHANNEL_1,
                                  /* Timer direct register
                                  .dier           = 0
};

*/


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void wheelPosSensorInit (void)
{
    /* First set all channels disabled */
    for (expchannel_t ch = 0; ch < EXT_MAX_CHANNELS; ch++ )
    {
        extcfg.channels[ch].mode  = EXT_CH_MODE_DISABLED;
        extcfg.channels[ch].cb    = NULL;
    }

    /* Start working EXT driver, current STM has only one driver */
    extStart( &EXTD1, &extcfg );

    /* Define channel config structure */
    EXTChannelConfig ch_conf;

    /* Fill in configuration for channel */
    ch_conf.mode  = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOF;
    ch_conf.cb    = extcb;

    /* Set channel (second arg) mode with filled configuration */
    extSetChannelMode( &EXTD1, 13, &ch_conf );
   // extSetChannelMode( &EXTD1, wheelPosSensorInLine, &ch_conf );

    palSetLineMode( wheelPosSensorInLine, PAL_MODE_INPUT_PULLUP );

    /* Start ICU driver */
    //icuStart( icuDriver, &icuCfg );
    /* Start capture */
    //icuStartCapture( icuDriver );
    /* Enable notifications (callbacks) */
    //icuEnableNotifications( icuDriver );


    gptStart( timeIntervalsDriver, &timeIntervalsCfg );
    /* 10ms trigger */
    gptStartContinuous( timeIntervalsDriver, TimerPeriod );


    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

}


/* Callback function of the EXT
 * It is triggered on event that is configured in config structure
 * args:  <extp>    - pointer to the driver, now it has
 *                    the only driver (EXTD1) and pointer will be like &EXTD1
 *        <channel> - channel where callback was triggered,
 *                    if trigger was from PD_0 - channel equals 0,
 *                    if from PB_3 - channel equals 3, like index in array
 */

uint32_t current_time = 0, prev_time = 0, time_interval = 0;
uint32_t measured_width = 0, overflow_counter = 0;

static void gpt_overflow_cb(GPTDriver *timeIntervalsDriver)
{
  /*if ( overflow_counter % 20 == 0 )
  {
    palToggleLine( LINE_LED2 );
  }*/

  overflow_counter ++;
}

static void extcb(EXTDriver *extp, expchannel_t channel)
{
/* The input arguments are not used now */
/* Just to avoid Warning from compiler */
    extp = extp; channel = channel;
    //palToggleLine( LINE_LED1 );
    impulseCounter ++;
    //current_time = gptGetCounterX(timeIntervalsDriver);
    //current_time = gptGetCounterX(timeIntervalsDriver)+ (overflow_counter-1)*TimerPeriod + (TimerPeriod - prev_time) ;
    measured_width = gptGetCounterX(timeIntervalsDriver)+ (overflow_counter-1)*TimerPeriod + (TimerPeriod - prev_time) ;
/*
    if ( prev_time > current_time )
    {
        measured_width = TimerPeriod - prev_time + current_time;
    }
    else
    {
        measured_width = current_time - prev_time;
    }

    //prev_time =  current_time;

    */
    prev_time =  gptGetCounterX(timeIntervalsDriver);
    overflow_counter = 0;
    //current_interval = gptGetIntervalX(timeIntervalsDriver);
}


/* Callback of ICU module, here used for width callback */
/* Keep measured width in ICU base timer ticks in the global variable */
/*
static void icu_width_cb ( ICUDriver *icup )
{
    /* Write new measured value
    measured_width = icuGetWidthX(icup);
    impulseCounter ++;
}*/

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
    velocity = ( PartOfWheelRevPerMinute * timeIntervalsCfg.frequency )/ ( measured_width );
    return velocity;
}

void sendTestInformation (void)
{
  wheelVelocity_t vel = wheelPosSensorGetVelocity ();
  chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n %s %d\r\n %s %d\r\n %s %d\r\n" ,
              "current time:", current_time,"prev time:", prev_time, "time width (tick):", measured_width,
              "velocity", vel, "ovflow", overflow_counter );

    //chprintf( (BaseSequentialStream *)&SD7, "%d\r\n %d\r\n",  vel, timeIntervalsCfg.frequency);
   // chThdSleepMilliseconds( 100 );
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
    position = impulseCounter/ImpsPerRevQuantity;
    return position;
}
