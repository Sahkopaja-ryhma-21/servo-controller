#include "spi.h"

#include <avr/io.h>         //peripheral control register definitions
#include <stdint.h>         //uint8_t and the like

//The SPI bus is used to receive commands and configuration data from the master controller.
//This initializes the SPI peripheral in slave mode.
void setupSpi()
{ 
    //SPI enabled, interrupts disabled SPI tranceiver as slave, MSb first, clock idle high, data sampled on clock leading edge, 250kHz
    SPCR0 = (0<<SPIE) | (1<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPR1) | (0<<SPR0); 
    SPSR0 = (0<<SPI2X);
}

//CS asserted = chip select is low
#define SPI_CS_ASSERTED (!(PINB & (1<<PINB2))) 
//Transfer complete = SPIF flag set in SPSR0.
//A read of the received byte is needed to clear the SPIF (SPI transfer complete) flag
#define SPI_TRANSFER_COMPLETE (SPSR0 & (1<<SPIF))

//Poll data from the SPI master and write the received bytes to a buffer. This function continues to
//..receive data until slave select is de-asserted (goes high). If more bytes than <bufSize> are
//..received, the extra bytes are ignored.
//
//  <*bytesReceived>:  how many bytes were received gets written to the variable referred by this pointer
//  <*buffer>:    the bytes received get written to the buffer referenced by this pointer
//  <bufSize>:    how many bytes are allowed to be received (bounds check)
void pollDataSpi(uint8_t *bytesReceived, uint8_t *buffer, uint8_t bufSize)
{
    (*bytesReceived) = 0;
    volatile uint8_t dummy = 0;

    //wait until chip select is asserted (is driven low by the master)
    while(!SPI_CS_ASSERTED)   

    //discard any junk already in the received data register.
    if(SPI_TRANSFER_COMPLETE) 
    {
        //A read of the received byte is needed to clear the SPIF (SPI transfer complete) flag, even 
        //..if there's no space left in the buffer. Otherwise the flag will stay set, which may mess something up.
        dummy = SPDR0;
        dummy; //<- this stops the compiler from complaining about unused variables.
    }

    //continue to receive data until chip select goes high
    while(SPI_CS_ASSERTED)    
    {
        if(SPI_TRANSFER_COMPLETE && (*bytesReceived) < bufSize)
        {
            buffer[*bytesReceived] = SPDR0;
            ++(*bytesReceived);
        }
    }
}