#ifndef INCLUDE_BLACK_BOX_H_
#define INCLUDE_BLACK_BOX_H_

/*
 * Hardware description
 * ------------------------------------
 * PAL driver       -> PD2 (CS for SPI)
 * SPI3 driver      -> PC10 (SCLK), PC11 (MISO), PC12 (MOSI)
 * Other            -> +5V, GND
 */

/**
 * @brief           Black box module intialization
 * @return  0       Intialized
 *          < 0     Intialization failed
 */
int blackBoxInit( void );

#endif /* INCLUDE_BLACK_BOX_H_ */
