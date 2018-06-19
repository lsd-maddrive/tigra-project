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

#define portTXSD4                   GPIOA
#define portRXSD4                   GPIOC
#define padTX4                      0
#define padRX4                      11

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

static const SerialConfig sd4cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

uint16_t brownSonarVal = 0;
uint8_t buf5Son[4];

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
         sdRead( &SD4, buf5Son, 3 );
         buf5Son[3] = 0;
         // convert bufSon into string, after this srt convert into long
         brownSonarVal = strtoul( buf5Son, NULL, 0 );
      }
    }
}

uint8_t buf7Son[4];
uint16_t greenSonarVal = 0;

static THD_WORKING_AREA(waGetSonarValU7Thd, 1024);
static THD_FUNCTION(GetSonarValU7Thd, arg)
{

    arg = arg;
    uint8_t firstR = 0;
    while( 1 )
    {
      firstR = sdGet( &SD7 );
      if( firstR == 'R' )
      {
         sdRead( &SD7, buf7Son, 3 );
         buf7Son[3] = 0;
         // convert bufSon into string, after this srt convert into long
         greenSonarVal = strtoul( buf7Son, NULL, 0 );
       }
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

    sdStart( &SD7, &sd7cfg );
    palSetPadMode( portSD7, padTX7, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portSD7, padRX7, PAL_MODE_ALTERNATE(8) );

    sdStart( &SD4, &sd4cfg );
    palSetPadMode( portTXSD4, padTX4, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portRXSD4, padRX4, PAL_MODE_ALTERNATE(8) );

    chThdCreateStatic( waGetSonarValU4Thd, sizeof(waGetSonarValU4Thd), NORMALPRIO, GetSonarValU4Thd, NULL ); // brown
    chThdCreateStatic( waGetSonarValU7Thd, sizeof(waGetSonarValU7Thd), NORMALPRIO, GetSonarValU7Thd, NULL ); // green

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
uint16_t getSonarValU7cm( void )
{

      return greenSonarVal;

}

