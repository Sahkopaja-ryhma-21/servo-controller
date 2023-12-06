#ifndef CONTROL_SYS_H
#define CONTROL_SYS_H

#include <stdint.h>         //uint8_t and the like
#include <avr/io.h>         //peripheral control register definitions

//This struct represents a PID (proportional–integral–derivative) controller. 
//https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
typedef struct
{
    int16_t setpoint;
    int16_t prevError;
    int16_t integral;
    uint8_t gainP;
    uint8_t gainI;
    uint8_t gainD;
} PidController;

//Initialize a PID controller
void initPid(volatile PidController *ctrl, uint8_t gainP, uint8_t gainI, uint8_t gainD);

//PID control loop implementation, intended to handle signed 10 bit values (matching the ADC resolution)
//The controller tries to drive the output in such a way that the feedback value approaches the setpoint.
inline int16_t updatePid(volatile PidController *ctrl, int16_t feedback)
{
    //The error is the distance from the setpoint.
    int16_t error      = ctrl->setpoint - feedback;

    //The derivative of the error represents how fast the error changes over time, while the integral
    //..accumulates error over time.
    //Normally one would of course multiply the error by deltaT when updating the integral, and divide the
    //..difference between samples by deltaT to calculate the derivative. This isn't done here in the interest
    //..of speed, which is mostly fine (but does mean that changing the update rate will detune the controller).
    ctrl->integral    += error; 
    int16_t derivative = error - ctrl->prevError;  
    ctrl->prevError   = error;                    

    //The integral is clamped to prevent integer overflow and the value wrapping around, which would
    //..have a catastrophic effect on the stability of the control system
    if(     ctrl->integral >  25000) ctrl->integral =  25000;
    else if(ctrl->integral < -25000) ctrl->integral = -25000;  

    //Calculate the output value. The AVR has a hardware multiplier, but no hardware division capability.
    //..Division by a constant which is a power of two simplifies to a bit shift. The integral term contains
    //..two such divisions to prevent integer overflow during multiplication.
    return(                                          //the output is the sum of
            error               * ctrl->gainP / 16 + //a proportional term
            ctrl->integral / 16 * ctrl->gainI / 4  + //an integral term
            derivative          * ctrl->gainD / 16   //a derivative term
        );
}

extern uint8_t  setupComplete;
extern uint8_t  calibrationComplete;
extern int16_t  angleCenter;
extern uint8_t  angleScale;
extern uint8_t  inverted;

//Angle control loop
extern volatile PidController angleController;
extern volatile uint8_t angleControllerEnable;
extern volatile int16_t angle;
extern volatile int16_t angleZero;
 
//Angular velocity control loop
extern volatile PidController velocityController;
extern volatile uint8_t velocityControllerEnable;
extern volatile int16_t velocity;
extern volatile int16_t velocityZero;

//Current (-> torque -> angular acceleration) control loop
extern volatile uint8_t coilInverted;
extern volatile PidController currentController;
extern volatile uint8_t currentControllerEnable;
extern volatile int16_t current;
extern volatile int16_t currentZero;

//How many clock cycles one cycle of the timer takes. With a F_CPU of 16 MHz and a timer frequency of 40 kHz
//..PWM_TIMER_PERIOD will be 400. That is, the timer will increment 400 times at 16 MHz before restarting.
#define PWM_TIMER_PERIOD (F_CPU / 40000UL)

//MAX_OUTPUT_LEVEL controls the maximum output duty cycle the coil is driven with
#define MAX_OUTPUT_LEVEL ((int16_t)PWM_TIMER_PERIOD / 2 - 20)

//The analog to digital converter is used to measure the angle and angular velocity with a 10 bit
//..resolution 10000 times per second. Coil current is also measured, but twice as often (20000
//..samples per second)
//Configure the analog to digital converter, and configure 16-bit timer 1 to start each 
//..ADC conversion at a precise point in time. 
//This doesn't start the timer yet, the timer will only start after startAdcPwm() is called.
void setupAdc();

//configure 16-bit timer 3 to generate two complementary (when one is high the other is low and vice versa)
//..pulse-width modulated square wave outputs. By controlling the coil driver, they determine the effective
//..output voltage.
//This doesn't start the timer yet, the timer will only start after startAdcPwm() is called.
void setupPwm();

//Stop the timers, synchronize them with a given phase shift, and start the timers.
//The "phaseShift" parameter is not an angle or percentage, but a difference in timer counts. It
//..determines how much ADC sampling will lead or lag the pulse-width modulation of the coil.
//A carefully chosen value will make sure that the ADC never takes a sample while the coil driver 
//..is changing state (and generating electrical noise as a consequence)
void startAdcPwm(int16_t phaseShift);

//Stop the ADC sampling timer and stop the PWM output timer. Also turns off voltage to the coil.
void stopAdcPwm();

//Set the output duty cycle (which controls the effective coil voltage)
inline void setPwm(int16_t outputLevel)
{
    if(     outputLevel >  MAX_OUTPUT_LEVEL) outputLevel =  MAX_OUTPUT_LEVEL;
    else if(outputLevel < -MAX_OUTPUT_LEVEL) outputLevel = -MAX_OUTPUT_LEVEL;
    OCR3A = OCR3B = (PWM_TIMER_PERIOD / 2) - outputLevel; //OCR3A and OCR3B are double buffered - new values come into effect only after the timer restarts from zero.
}

//Measure the average current over <averages> samples at zero voltage, and store the result in
//..currentZero. This value is then used to null out any offset in the measured current.
//Must be called with the PWM outputs on, but with the control loops disabled.
void currentZeroCalibration(uint16_t averages);

//Measure the average angular velocity over <averages> samples at zero voltage (and thus zero 
//..current, zero torque and zero velocity), and store the result in currentZero. This value is then 
//..used to null out any offset in the measured velocity.
//Must be called with the PWM outputs on, but with the control loops disabled.
void velocityZeroCalibration(uint16_t averages);

void angleZeroCalibration(uint16_t averages, int16_t outputLevel);

#endif//CONTROL_SYS_H