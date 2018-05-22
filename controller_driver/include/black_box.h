#ifndef INCLUDE_BLACK_BOX_H_
#define INCLUDE_BLACK_BOX_H_

#include <common.h>

/*
 * Hardware description
 * ------------------------------------
 * PAL driver       -> PD2 (CS for SPI)
 * SPI3 driver      -> PC10 (SCLK), PC11 (MISO), PC12 (MOSI)
 * Other            -> +5V, GND
 */

/**
 * @brief           Black box module intialization
 * @return  EOK     Intialized
 */
int blackBoxInit( void );

/**
 * @brief           Connect to black box SD card
 * 
 * @return  EOK     Connected
 *          EIO     SPI bus connection failed
 *          EFAULT  FS mount failed
 */
int blackBoxCardConnect( void );

/**
 * @brief           Disconnect from SD card
 */
void blackBoxCardDisconnect( void );

/*** TODO - Undocumented ***/

int blackBoxWriteData( void );

int blackBoxListFiles( BaseSequentialStream *chp, char *path );

#endif /* INCLUDE_BLACK_BOX_H_ */
