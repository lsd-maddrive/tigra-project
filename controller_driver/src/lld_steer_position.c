/*===========================================================================*/
/* ADC driver related.                                                       */
/*===========================================================================*/

#define ADC1_NUM_CHANNELS   2
#define ADC1_BUF_DEPTH      1

static adcsample_t samples1[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */

// when ADC conversion ends, this func will be called
/* if the depth is equal to 1, another way it will be called twice per conversion!
   conversion is sampling of all your cnahhel N times (N is buffer depth) */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
  (void)adcp;

  chSysLockFromISR();                   // Critical Area
  chMBPostI( &test_mb, samples1[0]);    // send 1st ADC-value (channel 1)
  chMBPostI( &test_mb, samples1[1]);    // send 2nd ADC-value (channel 2)
  chSysUnlockFromISR();                 // Close Critical Area

}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

static const ADCConversionGroup adcgrpcfg1 = {
  .circular     = true,                     // working mode = looped
  /* Buffer will continue writing to the beginning when it come to the end */
  .num_channels = ADC1_NUM_CHANNELS,    // number of channels
  .end_cb       = adccallback,              // after ADC conversion ends - call this func
  /* Don`t forget about depth of buffer */
  .error_cb     = adcerrorcallback,         // in case of errors, this func will be called
  .cr1          = 0,                        // just because it has to be 0
  /* Cause we don`t need to write something to the register */
  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12),  // Commutated from GPT
  /* 12 means 0b1100, and from RM (p.449) it is GPT4 */
  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),       // for AN10 - 144 samples
  .smpr2        = ADC_SMPR2_SMP_AN3(ADC_SAMPLE_144),        // for AN3  - 144 samples
  .sqr1         = ADC_SQR1_NUM_CH(ADC1_NUM_CHANNELS),   //
  /* Usually this field is set to 0 as config already know the number of channels (.num_channels) */
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) |         // sequence of channels
                  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10)
  /* If we can macro ADC_SQR2_SQ... we need to write to .sqr2 */
};

