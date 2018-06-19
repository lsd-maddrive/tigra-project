#include <tests.h>
#include <lld_steer_sensors.h>


/*** Hardware configuration     ***/

static GPTDriver                    *trSonarGPT            = &GPTD4;

#define portGreenSonar              GPIOG
#define padGreenSonar               2
#define portBrownSonar              GPIOG
#define padBrownSonar               3
#define portSonar                   GPIOG

#define portSD7                     GPIOE
#define padTX7                      8
#define padRX7                      7

#define portTXSD5                   GPIOC
#define portRXSD5                   GPIOD
#define padTX5                      12
#define padRX5                      2

static virtual_timer_t reset_vt;
static uint32_t port_arg;

/***  Reset pals to generate kick for sonars ***/
static void reset_cb(void *arg)
{

    palClearPad( portSonar, port_arg );

}

/*
 * @brief                   In callback we turn on sonars one by one
 */
uint8_t countGPT = 0;
static void gpt4cb( GPTDriver *gptp )
{

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

static const SerialConfig sd7cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

static const SerialConfig sd5cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

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

    sdStart( &SD7, &sd7cfg );
    palSetPadMode( portSD7, padTX7, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portSD7, padRX7, PAL_MODE_ALTERNATE(8) );

    sdStart( &SD5, &sd5cfg );
    palSetPadMode( portTXSD5, padTX5, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portRXSD5, padRX5, PAL_MODE_ALTERNATE(8) );

}

/*
 * @brief                   Receive values of sonar (in cm) through UART5
 * @arg                     firstR - first byte from sonar, if sonar works correctly firstR = 'R'
 * @arg                     buf - buffer name (size = 4 byte)
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU5cm( uint8_t firstR, uint8_t buf[4] )
{
    uint16_t sonarVal = 0;
    firstR = sdGet( &SD5 );
    if( firstR == 'R' )
    {
      sdRead( &SD5, buf, 3 );
      buf[3] = 0;
      // convert bufSon into string, after this srt convert into long
      sonarVal = strtoul( buf, NULL, 0 );
      return sonarVal;
    }
}

/*
 * @brief                   Receive values of sonar (in cm) through UART5
 * @arg                     firstR - first byte from sonar, if sonar works correctly firstR = 'R'
 * @arg                     buf - buffer name (size = 4 byte)
 * @return                  values of sonar in cm
 */
uint16_t getSonarValU7cm( uint8_t firstR, uint8_t buf[4] )
{
    uint16_t sonarVal = 0;
    firstR = sdGet( &SD7 );
    if( firstR == 'R' )
    {
      sdRead( &SD7, buf, 3 );
      buf[3] = 0;
      // convert bufSon into string, after this srt convert into long
      sonarVal = strtoul( buf, NULL, 0 );
      return sonarVal;
    }
}

