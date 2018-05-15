#include <lld_wheel_pos_sensor.h>


#define wheelPosSensorInLine       PAL_LINE(GPIOE, 9)

static void extcb(EXTDriver *extp, expchannel_t channel);
static EXTConfig extcfg;
uint32_t impulseCounter = 0;

static GPTDriver                        *timeIntervalsDriver = &GPTD3;
static const GPTConfig timeIntervalsCfg = {
                                             .frequency      =  1000000,// 1 MHz
                                             .callback       =  NULL,
                                             .cr2            =  TIM_CR2_MMS_1,
                                             .dier           =  0U
};

static void icu_width_cb(ICUDriver *icup);
static ICUDriver                     *icuDriver =   &ICUD1;
icucnt_t measured_width = 0;
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
                                   */
                                  .channel        = ICU_CHANNEL_1,
                                  /* Timer direct register */
                                  .dier           = 0
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
    ch_conf.mode  = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA;
    ch_conf.cb    = extcb;

    /* Set channel (second arg) mode with filled configuration */
    extSetChannelMode( &EXTD1, 7, &ch_conf );

    palSetLineMode( wheelPosSensorInLine, PAL_MODE_INPUT_PULLUP );
    /* Start ICU driver */
    icuStart( icuDriver, &icuCfg );
    /* Start capture */
    icuStartCapture( icuDriver );
    /* Enable notifications (callbacks) */
    icuEnableNotifications( icuDriver );


    gptStart( timeIntervalsDriver, &timeIntervalsCfg );
    /* 10ms trigger */
    gptStartContinuous( timeIntervalsDriver, 10000 );


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
    //palToggleLine( LINE_LED1 );
    impulseCounter ++;
}

/* Callback of ICU module, here used for width callback */
/* Keep measured width in ICU base timer ticks in the global variable */
static void icu_width_cb ( ICUDriver *icup )
{
    /* Write new measured value */
    measured_width = icuGetWidthX(icup);
    impulseCounter ++;
}

/**
 * @ brief                            Gets wheel current velocity value
 *                                    [revolutions per minute (rpm)]
 * @ param[in] ImpsPerRevQuantity     Number of impulses per revolution
 *                                    depends on given sensor
 * @ return                           Current wheel velocity value [rpm]
 *
 */
wheelVelocity wheelPosSensorGetVelocity ( uint16_t ImpsPerRevQuantity )
{
    wheelVelocity  velocity = 0;
    velocity = ( 60 * ( 1 / ImpsPerRevQuantity ) ) / S2ST(measured_width);
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
wheelPosition wheelPosSensorGetPosition ( uint16_t ImpsPerRevQuantity )
{
    wheelPosition position = 0;
    position = impulseCounter/ImpsPerRevQuantity;
    return position;
}
