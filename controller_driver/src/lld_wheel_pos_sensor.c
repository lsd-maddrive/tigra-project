#include <lld_wheel_pos_sensor.h>



static void extcb(EXTDriver *extp, expchannel_t channel);
static EXTConfig extcfg;
uint32_t impulseCounter = 0;

static GPTDriver                *timeIntervalsDriver   = &GPTD3;
static const GPTConfig timeIntervals_cfg = {
    .frequency      =  1000000,         // 1 MHz
    .callback       =  NULL,
    .cr2            =  TIM_CR2_MMS_1,
    .dier           =  0U
};


/*===========================================================================*/
/* Application code.                                                         */
/*===========================================================================*/
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

    gptStart( timeIntervalsDriver, &timeIntervals_cfg );
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
    palToggleLine( LINE_LED1 );
    impulseCounter ++;
}
