#include "configuration.h"

#include <stdint.h>         //uint8_t and the like
#include <avr/eeprom.h>     //configuration loading and storing

#include "controlSys.h"

//Default, hard coded configuration
const uint8_t defaultConfig[NUM_STORED_PARAMETERS] = {
    //PID controller gains for the three control loops.
    //P    I    D
      7,   8,   2, //Parameters 0...2: Current (indirectly torque/acceleration) controller 
     20,   0,  30, //Parameters 3...5: Velocity controller 
      8,   0,  30, //Parameters 6...8: Angle controller 

    //Input scaling and centering
    128,           //Parameter  9:     Center position of projected image. 128 = exactly centered.
    128,           //Parameter 10:     Size of projected image. 256 = 1/1, 128 = 1/2
      0            //Parameter 11:     Invert image if non-zero.
};

//Get the EEPROM address of the selected configuration slot. No bounds checking!
static void *slotAddress(ConfigSlot slotNumber)
{
    return (void *)( //<- explicit cast to void* to prevent the compiler from throwing a fit
            1 +      //<- because the first byte in EEPROM stores the slot number of the last accessed configuration
            (slotNumber - CONFIG_SLOT_1) * NUM_STORED_PARAMETERS
        );
}

//Get the identifier (slot number) of the last accessed (load or stored) configuration
//..the number is stored in non-volatile memory (EEPROM)
ConfigSlot getLatestSlot()
{
    eeprom_busy_wait();
    ConfigSlot slot = eeprom_read_byte((void *) 0);

    //bounds check in case the data is junk (when the EEPROM has been cleared, for example)
    if(slot >= CONFIG_SLOT_1 && slot <= CONFIG_SLOT_4)
        return slot;
    else
        return CONFIG_DEFAULT;
}


//Store the current state of the system to non-volatile memory
//This also updates the latest slot value value returned by getLatestSlot
//
//  <slotNumber> is the memory slot in EEPROM the data is written to
void storeConfig(ConfigSlot slotNumber)
{
    if(slotNumber == CONFIG_DEFAULT)
    {
        //don't write anything, the default config can't be changed.

        //store the identifier of the last accessed configuration
        eeprom_busy_wait(); 
        eeprom_write_byte((void *) 0, slotNumber);  
    }
    else if(slotNumber >= CONFIG_SLOT_1 && slotNumber <= CONFIG_SLOT_4)
    {
        //Serialize and store current system state to a buffer
        uint8_t configBuffer[NUM_STORED_PARAMETERS];
        for(uint8_t n = 0; n < NUM_STORED_PARAMETERS; ++n)
            configBuffer[n] = updateParameter(n, 0, 0);
        
        //Store configuration data from buffer to EEPROM 
        eeprom_busy_wait();
        eeprom_write_block(configBuffer, slotAddress(slotNumber), NUM_STORED_PARAMETERS);

        //store the identifier of the last accessed configuration
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber); 
    }
}

//Load a configuration from non-volatile memory and apply it to the current working values. 
//..This performs a rudimentary validity check: if all bytes are 0xFF (255), the memory is 
//..deduced to have been cleared and thus the default configuration is read instead.
//This also updates the latest slot value value returned by getLatestSlot
//
//  <slotNumber> is the memory slot the data is read from
void loadConfig(ConfigSlot slotNumber)
{
    if(slotNumber == CONFIG_DEFAULT)
    {
        //Update current system state with the default configuration
        for(uint8_t n = 0; n < NUM_STORED_PARAMETERS; ++n)
            updateParameter(n, defaultConfig[n], 1);

        //store the identifier of the last accessed configuration
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber); 
    }
    else if(slotNumber >= CONFIG_SLOT_1 && slotNumber <= CONFIG_SLOT_4)
    {
        //Read configuration data from EEPROM to buffer
        uint8_t configBuffer[NUM_STORED_PARAMETERS];
        eeprom_busy_wait();
        eeprom_read_block(configBuffer, slotAddress(slotNumber), NUM_STORED_PARAMETERS);

        //check the configuration for validity. Uninitialized values will read as 0xFF (255).
        uint8_t unitializedCount = 0;
        for(uint8_t n = 0; n < NUM_STORED_PARAMETERS; ++n)
            unitializedCount += (configBuffer[n] == 255);

        if(unitializedCount == NUM_STORED_PARAMETERS)
        {
            //if all values were unitialized, load the default configuration
            for(uint8_t n = 0; n < NUM_STORED_PARAMETERS; ++n)
                updateParameter(n, defaultConfig[n], 1);
        }
        else
        {
            //De-serialize buffer contents and modify current system state
            for(uint8_t n = 0; n < NUM_STORED_PARAMETERS; ++n)
                updateParameter(n, configBuffer[n], 1);
        }

        //store the identifier of the last accessed configuration
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber); 
    }
}


//Set the initialization state of the servo controller. This is used to turn the control system on
//..or off, optionally in various stages.
//
//  <value> See enum InitState in header for explanation
void setInitState(InitState value)
{
    if(value >= INIT_OUTPUT_ON) 
    {
        if(!setupComplete)
        {
            setupAdc();
            setupPwm();
            startAdcPwm(64); //ADC sampling precedes PWM transition by 64 clocks (4 us) to avoid noise
            setupComplete = 1;
        }
    }
    else
    {
        stopAdcPwm();
        setupComplete = 0;
    }
    if(value >= INIT_INPUT_CAL) 
    {
        if(!calibrationComplete)
        {
            currentZeroCalibration(256);
            velocityZeroCalibration(256);
            angleZeroCalibration(256, 100);
            calibrationComplete = 1;
        }
    }
    else
        calibrationComplete = 0;
    currentControllerEnable  = value >= INIT_CURRENT_CTRL; 
    velocityControllerEnable = value >= INIT_VELOCITY_CTRL;
    angleControllerEnable    = value >= INIT_ANGLE_CTRL; 
}

//Decode a parameter address and perform an action. This is used to serialize and deserialize the
//..state of the system in order to read and store configuration data, or to receive and transmit
//..over SPI.
//
//  <parameter>    The address of the desired parameter. See enum "Parameter" for reference
//  <value>        The value to be written to <parameter>, if writeEnable is true. Otherwise <value> 
//                 ..is ignored.
//  <writeEnable>  If true, this function will write to <parameter> and return its new value. If 
//                 ..false, this function will just return the value.
uint8_t updateParameter(Parameter parameter, uint8_t value, uint8_t writeEnable)
{
    switch (parameter)
    {
    //Accessor interfaces for PID controller gain values (see PID controller)
    case PARAM_CURRENT_GAIN_P:
        if(writeEnable)
            currentController.gainP = value;
        return currentController.gainP;
        
    case PARAM_CURRENT_GAIN_I:
        if(writeEnable)
        {
            currentController.integral = 0; //set the integral to zero to prevent wacky overshoots
            currentController.gainI = value;
        }
        return currentController.gainI;

    case PARAM_CURRENT_GAIN_D:
        if(writeEnable)
            currentController.gainD = value;
        return currentController.gainD;

    case PARAM_VELOCITY_GAIN_P:
        if(writeEnable)
            velocityController.gainP = value;
        return velocityController.gainP;
        
    case PARAM_VELOCITY_GAIN_I:
        if(writeEnable)
        {
            velocityController.integral = 0; //set the integral to zero to prevent wacky overshoots
            velocityController.gainI = value;
        }
        return velocityController.gainI;

    case PARAM_VELOCITY_GAIN_D:
        if(writeEnable)
            velocityController.gainD = value;
        return velocityController.gainD;
    
    case PARAM_ANGLE_GAIN_P:
        if(writeEnable)
            angleController.gainP = value;
        return angleController.gainP;
        
    case PARAM_ANGLE_GAIN_I:
        if(writeEnable)
        {
            angleController.integral = 0; //set the integral to zero to prevent wacky overshoots
            angleController.gainI = value;
        }
        return angleController.gainI;

    case PARAM_ANGLE_GAIN_D:
        if(writeEnable)
            angleController.gainD = value;
        return angleController.gainD;

    //Accessor interfaces for general parameters

    //This parameter is used to set the center of the projected image.
    //..when receiving an angle setpoint, a mid scale command (128) ends up at this setpoint.
    case PARAM_ANGLE_CENTER:
        if(writeEnable)
            angleCenter = (int16_t)value * 4 - 512; //map 8 bit unsigned to 10 bit signed
        return (angleCenter + 512) / 4;

    //This parameter is used to set the scale of the projected image.
    //a value of 0 causes the beam to always end up at the center. A value of 255 causes the beam to move
    //..to the greatest possible extent (1024 steps, not recommended as the servo may try to move out of
    //..bounds.
    case PARAM_ANGLE_SCALE:
        if(writeEnable)
            angleScale = value;
        return angleScale;

    //A non-zero value written here will invert the direction of motion.
    //Effectively reverses the sign of any read or written angle, velocity or current data (actual
    //..values and setpoints)
    //Angle scaling and centering is done before the inversion.
    case PARAM_INVERTED:
        if(writeEnable)
            inverted = (value != 0);
        return inverted;

    //This parameter is used to directly set the desired coil voltage. Only has an effect if the current 
    //..control loop is disabled.
    case PARAM_VOLTAGE_OUTPUT:
        if(!currentControllerEnable)
        {
            int16_t voltage = ((int32_t)value - 128) * MAX_OUTPUT_LEVEL / 128;
            if(inverted ^ coilInverted)
                setPwm(-voltage);
            else
                setPwm(voltage);
        }
        return 0; //TODO: implement reading

    //This parameter is used to directly set the desired coil current. Only has an effect if the velocity 
    //..control loop is disabled.
    case PARAM_CURRENT_SETPOINT:
        if(!velocityControllerEnable)
        {
            if(inverted ^ coilInverted)
                currentController.setpoint = 512 - (int16_t)value * 4; //map 8 bit unsigned value to 10 bit signed
            else
                currentController.setpoint = (int16_t)value * 4 - 512;
        }
        return 0; //TODO: implement reading

    //This parameter is used to set the desired velocity of the mirror. Only has an effect if the angle
    //..control loop is disabled.
    case PARAM_VELOCITY_SETPOINT:
        if(!angleControllerEnable)
        {
            if(inverted)
                velocityController.setpoint = 512 - (int16_t)value * 4; //map 8 bit unsigned value to 10 bit signed
            else
                velocityController.setpoint = (int16_t)value * 4 - 512; //map 8 bit unsigned value to 10 bit signed
        }
        return 0; //TODO: implement reading

    //This parameter is used to set the desired angle of the mirror. The given setpoint is scaled and centered
    //..as specified by PARAM_ANGLE_SCALE and PARAM_ANGLE_CENTER
    case PARAM_ANGLE_SETPOINT:
        {
            //scale the input value (in the range [0 ... 255]) to match the resolution of the measured angle 
            //..(in the range [-512 ... 511])
            int16_t setpoint = ((int16_t)value - 128) * angleScale / 64;

            //invert the scaled setpoint, if the option is set
            if(inverted)
                setpoint = -setpoint;

            //apply a constant offset (angleCenter) to the scaled setpoint. This can be used to steer the image.
            setpoint += angleCenter;

            //Bounds check to stop the setpoint from trying to drive the shaft out of bounds; limit
            //..movement to 90% of input range
            if(setpoint > 512 - 50)
                setpoint = 512 - 50;
            else if(setpoint < 50 - 512)
                setpoint = 50 - 512;

            //update the working value in the control loop
            angleController.setpoint = setpoint;
            
        }
        return 0; //TODO: implement reading, which is the same computation in reverse

    //TODO: reading actual values for current, angular velocity and angle

    case PARAM_STORE_CONFIG:
        storeConfig(value);
        return 0; 

    case PARAM_LOAD_CONFIG:
        loadConfig(value);
        return 0;

    //Set the initialization state of the servo controller. A value over 4 fully initializes everything.
    case PARAM_INIT_STATE:
        setInitState(value);
        return 0; //TODO: implement reading
    
    default:
        return 0;
    }
}