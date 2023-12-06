#include "configuration.h"

#include <stdint.h>         //uint8_t and the like
#include <avr/eeprom.h>     //configuration loading and storing

#include "controlSys.h"

const uint8_t defaultConfig[] = {
    //PID controller gains for the three control loops.
    //P    I    D
      7,   8,   2, //Parameters 0...2: Current controller 
     20,   0,  30, //Parameters 3...5: Velocity controller 
      8,   0,  30, //Parameters 6...8: Angle controller 

    //Input scaling and centering
    128,           //Parameter  9:     Center position of projected image. 128 = exactly centered.
    128,           //Parameter 10:     Size of projected image. 256 = 1/1, 128 = 1/2
      0            //Parameter 11:     Invert image if non-zero.
};

void storeConfig(ConfigSlot slotNumber)
{
    if(slotNumber == CONFIG_DEFAULT)
    {
        //don't write anything, the default config can't be changed.

        eeprom_busy_wait(); 
        eeprom_write_byte((void *) 0, slotNumber);
    } else if(slotNumber >= CONFIG_SLOT_1 && slotNumber <= CONFIG_SLOT_4)
    {
        uint8_t configBuffer[sizeof(defaultConfig)];
        void *slotAddress = (void *)(
                1 + (slotNumber - CONFIG_SLOT_1) * sizeof(configBuffer)
            );
        for(uint8_t n = 0; n < sizeof(configBuffer); ++n)
            configBuffer[n] = updateParameter(n, 0, 0);
        eeprom_busy_wait();
        eeprom_write_block(configBuffer, slotAddress, sizeof(configBuffer));
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber);
    }
}

void loadConfig(ConfigSlot slotNumber)
{
    if(slotNumber == CONFIG_DEFAULT)
    {
        for(uint8_t n = 0; n < sizeof(defaultConfig); ++n)
            updateParameter(n, defaultConfig[n], 1);
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber);
    }
    else if(slotNumber >= CONFIG_SLOT_1 && slotNumber <= CONFIG_SLOT_4)
    {
        uint8_t configBuffer[sizeof(defaultConfig)];
        void *slotAddress = (void *)(
                1 + (slotNumber - CONFIG_SLOT_1) * sizeof(configBuffer)
            );
        eeprom_busy_wait();
        eeprom_read_block(configBuffer, slotAddress, sizeof(configBuffer)); 
        for(uint8_t n = 0; n < sizeof(configBuffer); ++n)
            updateParameter(n, configBuffer[n], 1);
        eeprom_busy_wait();
        eeprom_write_byte((void *) 0, slotNumber);
    }
}

ConfigSlot getLatestSlot()
{
    eeprom_busy_wait();
    ConfigSlot slot = eeprom_read_byte((void *) 0);
    if(slot >= CONFIG_SLOT_1 && slot <= CONFIG_SLOT_4)
        return slot;
    else
        return CONFIG_DEFAULT;
}

void setInitState(InitState value)
{
    if(value >= INIT_OUTPUT_ON) 
    {
        if(!setupComplete)
        {
            //setupControllers();
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

uint8_t updateParameter(Parameter parameter, uint8_t value, uint8_t writeEnable)
{
    switch (parameter)
    {
    //PID controller gain values
    case PARAM_CURRENT_GAIN_P:
        if(writeEnable)
            currentController.gainP = value;
        return currentController.gainP;
        
    case PARAM_CURRENT_GAIN_I:
        if(writeEnable)
            currentController.gainI = value;
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
            velocityController.gainI = value;
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
            angleController.gainI = value;
        return angleController.gainI;

    case PARAM_ANGLE_GAIN_D:
        if(writeEnable)
            angleController.gainD = value;
        return angleController.gainD;

    //This parameter is used to set the center of the projected image.
    //..when receiving an angle setpoint, a mid scale command (128) ends up at this setpoint.
    case PARAM_ANGLE_CENTER:
        if(writeEnable)
            angleCenter = (int16_t)value * 4 - 512; //map 8 bit unsigned to 10 bit unsigned
        return (angleCenter + 512) / 4;

    //This parameter is used to set the scale of the projected image.
    //a value of 0 causes the beam to always end up at the center. A value of 255 causes the beam to move
    //..to the greatest possible extent (1024 steps, not recommended)
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
            int16_t scaledTo10bit = ((int16_t)value - 128) * angleScale / 64;
            if(inverted)
                angleController.setpoint = angleCenter - scaledTo10bit;
            else
                angleController.setpoint = angleCenter + scaledTo10bit;
        }
        return 0; //TODO: implement reading

    case PARAM_STORE_CONFIG:
        storeConfig(value);
        return 0; //TODO: implement reading

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