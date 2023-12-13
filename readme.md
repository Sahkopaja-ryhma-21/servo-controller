# Servo controller

This code is used to controll a qustombuilt servo. It takes commands from a SPI interface.

The commands go as follows:

If CS(chip select) is pulled up after one byte. The byte is interpreted as a position (from 0 to 255) where the servo will move to and try to hold.

If multiple bytes are given then the following happens.

The first byte is the parameter to be modified:

```c
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
```

The second byte is the value the corresponding parameter will be set to. If more bytes are given, the next parameter is set to that value. For example sending

```
CS DOWN
30
5
0
CS UP
```

Sets parameter 30 to a value of 5, and then parameter 31 to a value of 0
