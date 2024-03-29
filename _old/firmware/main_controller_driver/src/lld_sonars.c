#include <tests.h>
#include <lld_steer_sensors.h>


#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SHARP)
/***      ADC related        ***/
#define adc3NumChannels             1
#define adc3BufDepth                1

static adcsample_t adc3Buffer[adc3NumChannels * adc3BufDepth];

static ADCDriver                    *adcSharp          = &ADCD3;
static GPTDriver                    *adcGPT            = &GPTD8;

#define SharpAnalogInput         ADC_CHANNEL_IN9

#define SharpAnalogLine          PAL_LINE( GPIOF, 3 )

uint16_t sharpADCval           = 0;

/*** Callback prototype ***/



/*
 * @brief                   get adc value from buffer and write to variable
 */
static void adc3cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{

    adcp = adcp; n = n;

    sharpADCval = adc3Buffer[0];

}

/*** Configuration structures ***/
static const GPTConfig gpt8cfg1 = {
  .frequency =  100000,         // 100 kHz
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

static const ADCConversionGroup adc3cfg = {
  .circular     = true,                     // working mode = looped
  .num_channels = adc3NumChannels,          // number of channels
  .end_cb       = adc3cb,                   // after ADC conversion ends - call this func
  .error_cb     = NULL,
  .cr1          = 0,
  .cr2          = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(0b111),  // Commutated from Timer
  /* from RM (p.452) it is TIM8 TRGO */
  /* ADC_CR2_EXTEN_RISING - means to react on the rising signal (front) */
  .smpr1        = 0,
  .smpr2        = ADC_SMPR2_SMP_AN9(ADC_SAMPLE_480),
  .sqr1         = ADC_SQR1_NUM_CH(adc3NumChannels),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(SharpAnalogInput)
};

/*
 * @brief                   Initialize periphery connected to IR-sensor
 */
void lldSharpInit( void )
{
  /*** ADC pins configuration ***/
     gptStart( adcGPT, &gpt8cfg1 );
     adcStart( adcSharp, NULL );
     palSetLineMode( SharpAnalogLine,     PAL_MODE_INPUT_ANALOG );
     adcStartConversion( adcSharp, &adc3cfg, adc3Buffer, adc3BufDepth);

     gptStartContinuous( adcGPT, 1000); // triggering each 10 ms => 100 Hz
}

/*
 * @brief                   Return ADC value from IR-sensor (refreshed val - 10 Hz)
 */
uint16_t lldSharpADCval( void )
{
  return sharpADCval;

}


#endif

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SONAR)
/*** Hardware configuration     ***/

static GPTDriver                    *trSonarGPT            = &GPTD4;

#define portGreenSonar              GPIOG
#define padGreenSonar               2
#define portBrownSonar              GPIOG
#define padBrownSonar               3
#define portSonar                   GPIOG

#define portSD5                     GPIOD
//#define padTX5                      12
#define padRX5                      2

//#define portTXSD4                   GPIOA
#define portRXSD4                   GPIOC
//#define padTX4                      0
#define padRX4                      11

/***      Virtual timer Related   ***/
static virtual_timer_t reset_vt;
static uint32_t port_arg;

/***  Reset pals to generate kick for sonars ***/
static void reset_cb(void *arg)
{
    arg = arg;

    palClearPad( portSonar, port_arg );

}

/*
 * @brief                   In callback we turn on sonars one by one
 */
uint8_t countGPT = 0;
static void gpt4cb( GPTDriver *gptp )
{
    gptp = gptp;

    countGPT += 1;
    if( countGPT == 1)
    {
      palSetPad( portGreenSonar, padGreenSonar );
      port_arg = padGreenSonar;

    }
    else if( countGPT == 2)
    {
      palSetPad( portBrownSonar, padBrownSonar );
      port_arg = padBrownSonar;

      countGPT = 0;
    }
    // use virtual timer to generate width = 1 ms, not 10 ms of timer
    chSysLockFromISR();
    chVTSetI(&reset_vt, MS2ST(1), reset_cb, NULL);
    chSysUnlockFromISR();
}

/*** Configuration structures ***/
static const GPTConfig gpt4cfg1 = {
  .frequency =  100000,         // 100 kHz
  .callback  =  gpt4cb,
  .cr2       =  TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event.        */
  .dier      =  0U
};

static const SerialConfig sd5cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

static const SerialConfig sd4cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

uint16_t brownSonarVal = 0;
uint8_t buf4Son[4];

static THD_WORKING_AREA(waGetSonarValU4Thd, 1024);
static THD_FUNCTION(GetSonarValU4Thd, arg)
{
    arg = arg;
    uint8_t firstR = 0;

    while( 1 )
    {
      firstR = sdGet( &SD4);
      if( firstR == 'R' )
      {
         sdRead( &SD4, buf4Son, 3 );
         buf4Son[3] = 0;
         // convert bufSon into string, after this srt convert into long
         brownSonarVal = strtoul( (char *)buf4Son, NULL, 0 );
      }
    }
}

uint8_t buf5Son[4];
uint16_t greenSonarVal = 0;

static THD_WORKING_AREA(waGetSonarValU5Thd, 1024);
static THD_FUNCTION(GetSonarValU5Thd, arg)
{

    arg = arg;
    uint8_t firstR = 0;
    while( 1 )
    {
      firstR = sdGet( &SD5 );
      if( firstR == 'R' )
      {
         sdRead( &SD5, buf5Son, 3 );
         buf5Son[3] = 0;
         // convert bufSon into string, after this srt convert into long
         greenSonarVal = strtoul( (char *)buf5Son, NULL, 0 );
       }
    }
}

#define SOFT_INV_INPUT  PAL_LINE ( GPIOC, 10 )
#define SOFT_INV_OUTPUT PAL_LINE ( GPIOC, 12 )

static void extcb(EXTDriver *extp, expchannel_t channel)
{
    /* The input arguments are not used now */
    /* Just to avoid Warning from compiler */
    extp = extp; channel = channel;

    if ( palReadLine( SOFT_INV_INPUT ) == PAL_HIGH )
    {
        palClearLine( SOFT_INV_OUTPUT );
    }
    else
    {
        palSetLine( SOFT_INV_OUTPUT );
    }
}

/*
 * @brief                   Initialize periphery connected to sonars
 */
void lldSonarsInit( void )
{

    /*** PAL pins configuration ***/
    palSetPadMode( portGreenSonar, padGreenSonar, PAL_MODE_OUTPUT_PUSHPULL );
    palSetPadMode( portBrownSonar, padBrownSonar, PAL_MODE_OUTPUT_PUSHPULL );
    chVTObjectInit(&reset_vt);
    gptStart( trSonarGPT, &gpt4cfg1 );

    gptStartContinuous( trSonarGPT, 10000);   // triggering each 100 ms => 10 Hz

    sdStart( &SD5, &sd5cfg );
//    palSetPadMode( portSD7, padTX7, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portSD5, padRX5, PAL_MODE_ALTERNATE(8) );

    sdStart( &SD4, &sd4cfg );
//    palSetPadMode( portTXSD4, padTX4, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portRXSD4, padRX4, PAL_MODE_ALTERNATE(8) );

    chThdCreateStatic( waGetSonarValU4Thd, sizeof(waGetSonarValU4Thd), NORMALPRIO, GetSonarValU4Thd, NULL ); // brown
    chThdCreateStatic( waGetSonarValU5Thd, sizeof(waGetSonarValU5Thd), NORMALPRIO, GetSonarValU5Thd, NULL ); // green


    /* Software inversion */
    EXTChannelConfig ch_conf;

    /* Fill in configuration for channel */
    ch_conf.mode  = EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC;
    ch_conf.cb    = extcb;

    commonExtDriverInit();

    /* Set channel (second arg) mode with filled configuration */
    extSetChannelMode( &EXTD1, 10, &ch_conf );
    palSetLineMode( SOFT_INV_INPUT, PAL_MODE_INPUT );
    palSetLineMode( SOFT_INV_OUTPUT, PAL_MODE_OUTPUT_PUSHPULL );
}

/*
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU4cm( void )
{

      return brownSonarVal;

}

/*
 * @brief                   Get sonar values from memory
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU5cm( void )
{

      return greenSonarVal;

}
#endif
