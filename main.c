//#include <avr/io.h>         //peripheral control register definitions
//#include <util/delay.h>     //_delay_ms() and _delay_us()
#include <stdint.h>         //uint8_t and the like
#include <avr/interrupt.h>  //interrupt vector definitions

#include "spi.h"
#include "controlSys.h"
#include "configuration.h"

int main(void)
{
    sei();       //enable interrupts
    setupSpi();

    loadConfig(getLatestSlot());

    uint8_t commandBuffer[16];
    uint8_t bytesReceived = 0;
    while(1)
    {
        //Wait until a new command is received from the SPI master
        pollDataSpi(&bytesReceived, commandBuffer, sizeof(commandBuffer));

        //If only a single byte of data was received, it gets interpreted as a positioning command
        if(bytesReceived == 1)
            updateParameter(PARAM_ANGLE_SETPOINT, commandBuffer[0], 1);
        else if(bytesReceived > 1)
        {
            //if multiple bytes were received, the first gets interpreted as the number of the first
            //..parameter to be written to
            Parameter startingParameter = commandBuffer[0];

            //subsequent bytes are treated as successive parameter values
            for(uint8_t n = 1; n < bytesReceived; ++n)
                updateParameter(startingParameter + n - 1, commandBuffer[n], 1);
        }
    }
}