#include <stdint.h>         //uint8_t and the like
#include <avr/interrupt.h>  //interrupt vector definitions

#include "spi.h"            //SPI driver 
#include "controlSys.h"     //The servo loop (measurement -> calculating response -> driving outputs)
#include "configuration.h"  //Everything related to managing the state of the system

//Program entry point when booting up.
int main(void)
{
    sei();       //Enable interrupts
    setupSpi();  //Configure the SPI hardware, which is used to receive commands from the master

    //Load the configuration from the last accessed configuration slot. If no configuration has been
    //..accessed yet, loads the default configuration.
    loadConfig(getLatestSlot());

    uint8_t commandBuffer[32];
    uint8_t bytesReceived = 0;

    //Main loop, which just receives and executes instructions received over SPI.
    //..The servo loop is not part of this main code path, but executes in interrupts instead.
    while(1)
    {
        //Wait until a new command has been received from the SPI master (until chip select 
        //goes low and then back high)
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