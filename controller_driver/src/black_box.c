#include <common.h>
#include <black_box.h>

#define blackBoxCSLine      PAL_LINE(GPIOD, 2)

static const SPIConfig hs_spicfg = {
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
     * If software attempts to write one of the “Not used” values, they are forced to the value “0111”(8-
     * bit).
     */
    .cr2        = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static const SPIDriver  *blackBoxDrv    = &SPID3;

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

    spiStart(&SPID2, &hs_spicfg);

    return 0;
}
