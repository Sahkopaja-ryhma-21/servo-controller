#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdint.h>         //uint8_t and the like

//Parameters identify various variables or functions in the system.
typedef enum {
    //The following parameters can be loaded or stored as configuration data
    PARAM_CURRENT_GAIN_P    =  0, //Coil current PID controller - proportional term gain
    PARAM_CURRENT_GAIN_I    =  1, //Coil current PID controller - integral term gain
    PARAM_CURRENT_GAIN_D    =  2, //Coil current PID controller - derivative term gain
    PARAM_VELOCITY_GAIN_P   =  3, //Angular velocity PID controller - proportional term gain
    PARAM_VELOCITY_GAIN_I   =  4, //Angular velocity PID controller - integral term gain
    PARAM_VELOCITY_GAIN_D   =  5, //Angular velocity PID controller - derivative term gain
    PARAM_ANGLE_GAIN_P      =  6, //Angle PID controller - proportional term gain
    PARAM_ANGLE_GAIN_I      =  7, //Angle PID controller - integral term gain
    PARAM_ANGLE_GAIN_D      =  8, //Angle PID controller - derivative term gain
    PARAM_ANGLE_CENTER      =  9, //Center position of projected image
    PARAM_ANGLE_SCALE       = 10, //How big the projected image is around the center position
    PARAM_INVERTED          = 11, //If true (non-zero), flips the sign of all setpoints (angle,
                                  //..angular velocity and current) and all actual values.

    //The following parameters are never loaded or stored as part of a configuration
    PARAM_VOLTAGE_OUTPUT    = 12, //Output voltage. Read only if init state > INIT_INPUT_CAL
    PARAM_CURRENT_SETPOINT  = 13, //Desired velocity. Read only if init state > INIT_CURRENT_CTRL
    PARAM_VELOCITY_SETPOINT = 14, //Desired velocity. Read only if init state > INIT_VELOCITY_CTRL
    PARAM_ANGLE_SETPOINT    = 15, //Desired angle. Read/write. 
    PARAM_CURRENT_ACTUAL    = 16, //Actual value of current. Read only.
    PARAM_VELOCITY_ACTUAL   = 17, //Actual value of angular velocity. Read only.
    PARAM_ANGLE_ACTUAL      = 18, //Actual value of angle. Read only.

    //... reserved for future use

    PARAM_STORE_CONFIG      = 29, //Writing to this stores configuration. The value written 
                                  //..determines which slot is overwritten. 
    PARAM_LOAD_CONFIG       = 30, //Writing to this loads configuration. The value written
                                  //..determines which slot is read.
    PARAM_INIT_STATE        = 31  //Set the initialization state of the system. See enum InitState
                                  //..and setInitState() for details.
} Parameter;

//how many of the parameters  (starting from 0), are stored in non-volatile memory as part of a
//..configuration 
#define NUM_STORED_PARAMETERS 12

//InitState defines the behavior of setInitState, which can be used to turn off or turn on the
//..system in granular fashion.
typedef enum{
    INIT_OFF            = 0, //Disable everything. Coil drivers off.
    INIT_OUTPUT_ON      = 1, //Initialize controllers and hardware peripherals. Coil drivers on, start ADC.
    INIT_INPUT_CAL      = 2, //Perform calibration procedures.
    INIT_CURRENT_CTRL   = 3, //Enable closed loop current control.
    INIT_VELOCITY_CTRL  = 4, //Enable closed loop angular velocity control.
    INIT_ANGLE_CTRL     = 5  //Enable closed loop angle control. System fully active.
} InitState;

//There are four configuration slots that can be read from or written to on the fly using storeConfig
//..or loadConfig
typedef enum{
    CONFIG_DEFAULT      = 0,
    CONFIG_SLOT_1       = 1,
    CONFIG_SLOT_2       = 2,
    CONFIG_SLOT_3       = 3,
    CONFIG_SLOT_4       = 4
} ConfigSlot;

//Get the identifier (slot number) of the last accessed (load or stored) configuration.
//..Like the configuration itself, the number is stored in non-volatile memory (EEPROM).
ConfigSlot getLatestSlot();

//Store the current state of the system to non-volatile memory
//This also updates the latest slot value value returned by getLatestSlot
//
//  <slotNumber> is the memory slot in EEPROM the data is written to
void storeConfig(ConfigSlot slotNumber);

//Load a configuration from non-volatile memory and apply it to the current working values. 
//..This performs a rudimentary validity check: if all bytes are 0xFF (255), the memory is 
//..deduced to have been cleared and thus the default configuration is read instead.
//This also updates the latest slot value value returned by getLatestSlot
//
//  <slotNumber> is the memory slot the data is read from
void loadConfig(ConfigSlot slotNumber);

//Set the initialization state of the servo controller. This is used to turn the control system on
//..or off, optionally in various stages.
//
//  <value> See enum InitState in header for explanation
void setInitState(InitState value);

//Decode a parameter address and perform an action. This is used to serialize and deserialize the
//..state of the system in order to read and store configuration data, or to receive and transmit
//..over SPI.
//
//  <parameter>    The address of the desired parameter. See enum "Parameter" for reference
//  <value>        The value to be written to <parameter>, if writeEnable is true. Otherwise <value> 
//                 ..is ignored.
//  <writeEnable>  If true, this function will write to <parameter> and return its new value. If 
//                 ..false, this function will just return the value.
uint8_t updateParameter(Parameter parameter, uint8_t value, uint8_t writeEnable);

#endif//CONFIGURATION_H