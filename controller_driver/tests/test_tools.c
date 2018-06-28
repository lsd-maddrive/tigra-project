#include <common.h>

/*************************************/
/*** MATLAB slider testing routine ***/
/*************************************/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

#define START_DATA  127
#define STOP_DATA   -127

bool    dataTransferEnabled = false;
int16_t controlValue        = 0;

static THD_WORKING_AREA(waSender, 128);
static THD_FUNCTION(Sender, arg) 
{
    arg = arg;

    while ( 1 )
    {
        if ( dataTransferEnabled )
            sdWrite( &SD7, (void *)&controlValue, sizeof( controlValue ) );

        chThdSleepMilliseconds( 100 );
    }
}

static THD_WORKING_AREA(waReceiver, 128);
static THD_FUNCTION(Receiver, arg) 
{
    arg = arg;

    while ( 1 )
    {
        /* Freezed until control received */
        int8_t rcv_data = sdGet( &SD7 );

        switch ( rcv_data )
        {
            case START_DATA:
                dataTransferEnabled = true;
                break;

            case STOP_DATA:
                dataTransferEnabled = false;
                break;

            default:
                controlValue = CLIP_VALUE( rcv_data, -100, 100 );
        }       

        chThdSleepMilliseconds( 100 );
    }
}

void testToolsMatlabSliderRoutine ( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    chThdCreateStatic( waSender, sizeof(waSender), NORMALPRIO, Sender, NULL );
    chThdCreateStatic( waReceiver, sizeof(waReceiver), NORMALPRIO, Receiver, NULL );

    while ( 1 )
    {
        /* Here we can set other tasks */

        chThdSleepMilliseconds( 100 );
    }

}
