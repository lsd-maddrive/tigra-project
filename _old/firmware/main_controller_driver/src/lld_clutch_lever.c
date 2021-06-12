#include <lld_clutch_lever.h>

static clutchLeverCb_t m_callback = NULL;

static void extcb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if ( m_callback )
        m_callback();
}

/**
 * @brief                   Initialization of clutch lever driver
 * @param[in] callback      Callback function that is called when clutch lever
 *                          is pressed (front)
 * @note                    Callback function is called inside ISR context
 */
void clutchLeverInit ( clutchLeverCb_t callback )
{
    EXTChannelConfig ch_conf = {
        .mode = EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA,
        .cb = extcb
    };

    commonExtDriverInit();

    extSetChannelMode( &EXTD1, 1, &ch_conf );

    m_callback = callback;
}
