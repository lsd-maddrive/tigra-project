#include <common.h>
#include <ff.h>
#include <black_box.h>

#define blackBoxCSLine      PAL_LINE(GPIOD, 2)

static const SPIConfig  hs_spicfg = {
    .end_cb     = NULL,
    .ssport     = PAL_PORT( blackBoxCSLine ),
    .sspad      = PAL_PAD( blackBoxCSLine ),
    /*      RM p.1357
     * BR[2:0]: Baud rate control
     * 000: fPCLK/2
     * 001: fPCLK/4
     * 010: fPCLK/8
     * 011: fPCLK/16
     * 100: fPCLK/32
     * 101: fPCLK/64
     * 110: fPCLK/128
     * 111: fPCLK/256
     * Note: These bits should not be changed when communication is ongoing.
     */
    .cr1        = SPI_CR1_BR_0,
    /*      RM p.1359
     * DS [3:0]: Data size
     * These bits configure the data length for SPI transfers:
     * 0000: Not used
     * 0001: Not used
     * 0010: Not used
     * 0011: 4-bit
     * 0100: 5-bit
     * 0101: 6-bit
     * 0110: 7-bit
     * 0111: 8-bit
     * 1000: 9-bit
     * 1001: 10-bit
     * 1010: 11-bit
     * 1011: 12-bit
     * 1100: 13-bit
     * 1101: 14-bit
     * 1110: 15-bit
     * 1111: 16-bit
     * If software attempts to write one of the “Not used” values, 
     * they are forced to the value “0111”(8-bit).
     */
    .cr2        = 0     // SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static const SPIConfig  ls_spicfg = {
    .end_cb     = NULL,
    .ssport     = PAL_PORT( blackBoxCSLine ),
    .sspad      = PAL_PAD( blackBoxCSLine ),
    .cr1        = SPI_CR1_BR_2 | SPI_CR1_BR_1,
    .cr2        = 0     // SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

/* MMC/SD over SPI driver configuration.*/
static const MMCConfig  mmccfg = { &SPID3, &ls_spicfg, &hs_spicfg };

MMCDriver   MMCD1;
MMCDriver   *blackBoxDrv    = &MMCD1;

FATFS       MMC_FS;

/**
 * @brief           Black box module intialization
 * @return  0       Intialized
 *          < 0     Intialization failed
 */
int blackBoxInit( void )
{
    /*
    * SPI2 I/O pins setup.
    */

    /* SPI SCK */
    palSetLineMode( PAL_LINE(GPIOC, 10U),
                    PAL_MODE_ALTERNATE(6) |
                    PAL_STM32_OSPEED_HIGHEST);
    /* MISO */
    palSetLineMode( PAL_LINE(GPIOC, 11U),
                    PAL_MODE_ALTERNATE(6) |
                    PAL_STM32_OSPEED_HIGHEST);
    /* MOSI */
    palSetLineMode( PAL_LINE(GPIOC, 12U),
                    PAL_MODE_ALTERNATE(6) |
                    PAL_STM32_OSPEED_HIGHEST);

    /* CS */
    palSetLine( blackBoxCSLine );
    palSetLineMode( blackBoxCSLine,
                    PAL_MODE_OUTPUT_PUSHPULL );

    /* MMC init */
    mmcObjectInit( blackBoxDrv );
    mmcStart( blackBoxDrv, &mmccfg );

    return 0;
}

int blackBoxIsCardInserted( void )
{
    BaseBlockDevice *bbdp = (BaseBlockDevice *)blackBoxDrv;

    blkstate_t state = blkGetDriverState( bbdp );

    if ((state != BLK_READING) && (state != BLK_WRITING)) 
    {
        return blkIsInserted(bbdp) ? 1 : 0;
    }

    return -1;
}

int blackBoxCardConnect( void )
{
    FRESULT err;

    if ( mmcConnect( blackBoxDrv ) )
        return -1;

    err = f_mount( &MMC_FS, "/", 0 );

    if (err != FR_OK) 
    {
        mmcDisconnect( blackBoxDrv );
        return -1;
    }

    return 0;
}


void blackBoxCardDisconnect( void )
{
    mmcDisconnect( blackBoxDrv );
}

#include <chprintf.h>
#include <string.h>

int blackBoxListFiles( BaseSequentialStream *chp, char *path )
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;


    res = f_opendir(&dir, path);
    if (res == FR_OK) 
    {
        i = strlen(path);
        for (;;) 
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;
            if (fno.fname[0] == '.')
                continue;
            fn = fno.fname;
            
            if (fno.fattrib & AM_DIR) 
            {
                path[i++] = '/';
                strcpy( &path[i], fn );
                res = blackBoxListFiles(chp, path);
                if (res != FR_OK)
                  break;
                path[--i] = 0;
            }
            else 
            {
                chprintf(chp, "%s/%s\n", path, fn);
            }
        }
    }
    
    return res;
}
