#ifndef SPI_H
#define SPI_H

#include <stdint.h>         //uint8_t and the like

//The SPI bus is used to receive commands and configuration data from the master controller.
//This initializes the SPI peripheral in slave mode.
void setupSpi();

//Poll data from the SPI master and write the received bytes to a buffer. This function continues to
//..receive data until slave select is de-asserted (goes high). If more bytes than <bufSize> are
//..received, the extra bytes are ignored.
//
//  <*bytesReceived>:  how many bytes were received gets written to the variable referred by this pointer
//  <*buffer>:    the bytes received get written to the buffer referenced by this pointer
//  <bufSize>:    how many bytes are allowed to be received (bounds check)
void pollDataSpi(uint8_t *bytesReceived, uint8_t *buffer, uint8_t bufSize);


#endif//SPI_H