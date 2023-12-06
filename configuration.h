#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdint.h>         //uint8_t and the like

typedef enum {
    PARAM_CURRENT_GAIN_P    =  0,
    PARAM_CURRENT_GAIN_I    =  1,
    PARAM_CURRENT_GAIN_D    =  2,
    PARAM_VELOCITY_GAIN_P   =  3,
    PARAM_VELOCITY_GAIN_I   =  4,
    PARAM_VELOCITY_GAIN_D   =  5,
    PARAM_ANGLE_GAIN_P      =  6,
    PARAM_ANGLE_GAIN_I      =  7,
    PARAM_ANGLE_GAIN_D      =  8,
    PARAM_ANGLE_CENTER      =  9,
    PARAM_ANGLE_SCALE       = 10,
    PARAM_INVERTED          = 11,
    PARAM_VOLTAGE_OUTPUT    = 12, //Current output voltage. Read only if init state > INIT_INPUT_CAL
    PARAM_CURRENT_SETPOINT  = 13, //Desired velocity. Read only if init state > INIT_CURRENT_CTRL
    PARAM_VELOCITY_SETPOINT = 14, //Desired velocity. Read only if init state > INIT_VELOCITY_CTRL
    PARAM_ANGLE_SETPOINT    = 15, //Desired angle. Read/write. 
    PARAM_CURRENT_ACTUAL    = 16, //Measured current. Read only.
    PARAM_VELOCITY_ACTUAL   = 17, //Measured velocity. Read only.
    PARAM_ANGLE_ACTUAL      = 18, //Measured angle. Read only.
    //... reserved for future use
    PARAM_STORE_CONFIG      = 29,
    PARAM_LOAD_CONFIG       = 30, 
    PARAM_INIT_STATE        = 31
} Parameter;

typedef enum{
    INIT_OFF            = 0, //Disable everything. Coil drivers off.
    INIT_OUTPUT_ON      = 1, //Initialize controllers and hardware peripherals. Coil drivers on, start ADC.
    INIT_INPUT_CAL      = 2, //Perform calibration procedures.
    INIT_CURRENT_CTRL   = 3, //Enable closed loop current control.
    INIT_VELOCITY_CTRL  = 4, //Enable closed loop angular velocity control.
    INIT_ANGLE_CTRL     = 5  //Enable closed loop angle control. System fully active.
} InitState;

typedef enum{
    CONFIG_DEFAULT      = 0,
    CONFIG_SLOT_1       = 1,
    CONFIG_SLOT_2       = 2,
    CONFIG_SLOT_3       = 3,
    CONFIG_SLOT_4       = 4
} ConfigSlot;

void storeConfig(ConfigSlot slotNumber);
void loadConfig(ConfigSlot slotNumber);
ConfigSlot getLatestSlot();
void setInitState(InitState value);
uint8_t updateParameter(Parameter parameter, uint8_t value, uint8_t writeEnable);

#endif//CONFIGURATION_H