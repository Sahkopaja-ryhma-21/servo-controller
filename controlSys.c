#include "controlSys.h"

#include <stdint.h>         //uint8_t and the like
#include <avr/io.h>         //peripheral control register definitions
#include <avr/interrupt.h>  //interrupt vector definitions
#include <util/delay.h>     //_delay_ms() and _delay_us()

uint8_t  setupComplete       =   0;
uint8_t  calibrationComplete =   0;
int16_t  angleCenter         = 512;
uint8_t  angleScale          = 127;
uint8_t  inverted            =   0;

//Angle control loop
volatile PidController angleController;
volatile uint8_t angleControllerEnable    = 0;
volatile int16_t angle                    = 0;
volatile int16_t angleZero                = 0;

//Angular velocity control loop
volatile PidController velocityController;
volatile uint8_t velocityControllerEnable = 0;
volatile int16_t velocity                 = 0;
volatile int16_t velocityZero             = 0;

//Current (-> torque -> angular acceleration) control loop
volatile uint8_t coilInverted             = 0;
volatile PidController currentController;
volatile uint8_t currentControllerEnable  = 0;
volatile int16_t current                  = 0;
volatile int16_t currentZero              = 0;

//Initialize a PID controller
void initPid(volatile PidController *ctrl, uint8_t gainP, uint8_t gainI, uint8_t gainD)
{
    ctrl->setpoint = 0; 
    ctrl->prevError = 0;
    ctrl->integral = 0;
    ctrl->gainP = gainP; //proportional gain
    ctrl->gainI = gainI; //integral gain
    ctrl->gainD = gainD; //derivative gain
}

//PID control loop implementation, intended to handle signed 10 bit values (matching the ADC resolution)
//The controller tries to drive the output in such a way that the feedback value approaches the setpoint.
extern inline int16_t updatePid(volatile PidController *ctrl, int16_t feedback);
//see header for implementation of inline functions

//How many clock cycles one cycle of the timer takes. With a F_CPU of 16 MHz and a timer frequency of 40 kHz
//..PWM_TIMER_PERIOD will be 400. That is, the timer will increment 400 times at 16 MHz before restarting.
#define PWM_TIMER_PERIOD (F_CPU / 40000UL)

//The analog to digital converter is used to measure the angle and angular velocity with a 10 bit
//..resolution 10000 times per second. Coil current is also measured, but twice as often (20000
//..samples per second)
//Configure the analog to digital converter, and configure 16-bit timer 1 to start each 
//..ADC conversion at a precise point in time. 
//This doesn't start the timer yet, the timer will only start after startAdcPwm() is called.
void setupAdc()
{
    //Timer1 configuration. Timer1 is used to trigger ADC sampling
    //In this timer mode, the timer value increments once every clock cycle (at 16 MHz) until the value
    //..reaches ICR1. Then the timer restarts from 0.
    ICR1   = PWM_TIMER_PERIOD;
    TCCR1A = (1<<WGM11)  | (0<<WGM10); //fast pwm mode, TOP = ICR1
    TCCR1B = (1<<WGM13)  | (1<<WGM12); 
    TIMSK1 = (1<<TOIE1); //timer 1 overflow interrupt enabled

    //Analog to digital converter configuration
    //AVCC with external capacitor at AREF pin, left adjust result off, MUX input set to ADC0
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0); 
    //ADC enabled, auto trigger enabled, ADC interrupts disabled, ADC prescaler 1:16 (1 MHz ADC clock)
    ADCSRA = (1<<ADEN) | (1<<ADATE) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0); 
    //ADC Auto Trigger Source: Timer/counter 1 oferflow. This is how ADC sampling is synchronized.
    ADCSRB = (1<<ADTS2) | (1<<ADTS1) | (0<<ADTS0);
    //disable digital input buffers for ADC4 (angle), ADC3 (angular velocity), ADC6 (current/torque/acceleration)
    DIDR0 = (1<<ADC4D) | (1<<ADC3D) | (1<<ADC6D);
}

//configure 16-bit timer 3 to generate two complementary (when one is high the other is low and vice versa)
//..pulse-width modulated square wave outputs. By controlling the coil driver, they determine the effective
//..output voltage.
//This doesn't start the timer yet, the timer will only start after startAdcPwm() is called.
void setupPwm()
{
    //In this timer mode, the timer value increments once every clock cycle (at 16 MHz) until the value
    //..reaches ICR3. Then the timer restarts from 0.
    ICR3   = PWM_TIMER_PERIOD; 

    //OCR3A and OCR3B control when the coil driver outputs are driven high or low. At half of the timer period
    //..the outputs spend an equal amount of time high and low, and no net current will flow.
    OCR3A  = PWM_TIMER_PERIOD / 2; 
    OCR3B  = PWM_TIMER_PERIOD / 2;

    //fast pwm mode, timer period is set by ICR3 (TOP = ICR3)
    TCCR3A = (1<<WGM31)  | (0<<WGM30); 
    TCCR3B = (1<<WGM33)  | (1<<WGM32);
}

//Stop the timers, synchronize them with a given phase shift, and start the timers.
//The "phaseShift" parameter is not an angle or percentage, but a difference in timer counts. It
//..determines how much ADC sampling will lead or lag the pulse-width modulation of the coil.
//A carefully chosen value will make sure that the ADC never takes a sample while the coil driver 
//..is changing state (and generating electrical noise as a consequence)
void startAdcPwm(int16_t phaseShift)
{
    //make sure that the outputs are off and the timers are stopped.
    stopAdcPwm();

    //set the timer counter values in a way that gets the desired phase shift
    if(phaseShift >= 0)
    {
        TCNT1  = phaseShift; 
        TCNT3  = 1; //timer 3 gets started after timer 1, so it gets a head start
    }
    else
    {
        TCNT1  = 0; 
        TCNT3  = 1 - phaseShift; //timer 3 gets started after timer 1, so it gets a head start
    }

    //Restart the timers
    uint8_t TCCR1B_temp = TCCR1B | (1<<CS10); //prescaler 1/1
    uint8_t TCCR3B_temp = TCCR3B | (1<<CS30); //prescaler 1/1
    cli(); //disable interrupts (time critical code)
    TCCR1B = TCCR1B_temp; //start the ADC sampling timer
    TCCR3B = TCCR3B_temp; //start the PWM output timer 1 clock cycle later.
    sei(); //re-enable interrupts

    //OC3A is connected to PD0 and OC3B to PD2, so set them as outputs
    DDRD  |= (1<<DDRD0) | (1<<DDRD2); 
    PORTD |= (1<<PORTD2); //<- from datasheet: For OC3B or OC4B when not using the Output Compare Modulator, PORTD2 must also be set in order to enable the output. 

    //Enable the PWM outputs.
    //On every timer cycle:
    TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | //start with output OC3A high, clear OC3A on compare match (when the count TCNT3 reaches OCR3A)
              (1<<COM3B1) | (0<<COM3B0);  //start with output OC3B low,  set   OC3B on compare match (when the count TCNT3 reaches OCR3B)
}

//Stop the ADC sampling timer and stop the PWM output timer. Also turns off voltage to the coil.
void stopAdcPwm()
{
    //Turn off the OC3A and OC3B PWM outputs.
    TCCR3A &= ~((1<<COM3A1) | (1<<COM3A0) | (1<<COM3B1) | (1<<COM3B0));

    //Drive the I/O pins low, which puts the motor driver IC in sleep mode
    DDRD  &= ~((1<<DDRD0) | (1<<DDRD2)); 
    PORTD &= ~((1<<PORTD0) | (1<<PORTD2)); 

    //Turn off the timers by setting the clock prescalers to 0, which removes their clock source
    TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10)); //ADC sampling timer prescaler
    TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30)); //PWM timer prescaler
}

//Set the output duty cycle (which controls the effective coil voltage)
extern inline void setPwm(int16_t outputLevel);
//see header for implementation of inline functions

//State of the ADC sampling logic
volatile enum {LATEST_VELOCITY, LATEST_CURRENT_1, LATEST_ANGLE, LATEST_CURRENT_2} samplingState = LATEST_VELOCITY;

//Timer 1 overflow interrupt.
//This code runs at 40000 times per second - every time a new ADC conversion gets started and the 
//..previous result becomes available.
//The code retrieves the latest ADC sampling result, and updates one of the three control loops 
//..(angle, angular velocity or current), depending on which of the three was just measured.
ISR(TIMER1_OVF_vect)
{
    switch (samplingState)
    {
      //latest conversion: velocity, now converting: current, setting up next conversion: angle
      case LATEST_VELOCITY:
        velocity = velocityZero - ADC; //The differentiator circuit inverts the signal, so it gets inverted here again to cancel that.
        //update the angular velocity PID controller
        if(velocityControllerEnable)
        {
            if(coilInverted)
                currentController.setpoint = -updatePid(&velocityController, velocity);
            else
                currentController.setpoint = updatePid(&velocityController, velocity);
        }  
        ADMUX = (ADMUX & ~(0b1111 << MUX0)) | (4 << MUX0); //Next input: (ADC4 / angle).
        samplingState = LATEST_CURRENT_1;  
        break;

      //latest conversion: current, now converting: angle, setting up next conversion: current
      case LATEST_CURRENT_1:
        //update the coil current PID controller
        current = currentZero - ADC; //The current input is inverted relative to the voltage output, so it gets inverted here again to cancel that.
        if(currentControllerEnable)
        {
            setPwm(updatePid(&currentController, current));
        }
        ADMUX = (ADMUX & ~(0b1111 << MUX0)) | (6 << MUX0); //Next input: (ADC6 / current).
        samplingState = LATEST_ANGLE;
        break;

      //latest conversion: angle, now converting: current, setting up next conversion: velocity
      case LATEST_ANGLE:
        //update the mirror angle PID controller
        angle = ADC - angleZero; 
        if(angleControllerEnable)
        {
            velocityController.setpoint = updatePid(&angleController, angle);
        }
        ADMUX = (ADMUX & ~(0b1111 << MUX0)) | (3 << MUX0); //Next input: (ADC3 / angular velocity).
        samplingState = LATEST_CURRENT_2;
        break;

      //latest conversion: current, now converting: velocity, setting up next conversion: current
      case LATEST_CURRENT_2:
        //update the coil current PID controller
        current = currentZero - ADC;
        if(currentControllerEnable)
        {
            setPwm(updatePid(&currentController, current));
        }
        ADMUX = (ADMUX & ~(0b1111 << MUX0)) | (6 << MUX0); //Next input: (ADC6 / current).
        samplingState = LATEST_VELOCITY;
        break;
    }
}

//Measure the average current over <averages> samples at zero voltage, and store the result in
//..currentZero. This value is then used to null out any offset in the measured current.
//Must be called with the PWM outputs on, but with the control loops disabled.
void currentZeroCalibration(uint16_t averages)
{
    setPwm(0);
    currentZero = 0;
    int32_t sum = 0;
    for(uint16_t n = 0; n < averages / 2; ++n)
    {
        while (samplingState != LATEST_CURRENT_1);
        sum -= current;
        while (samplingState != LATEST_CURRENT_2);
        sum -= current;
    }
    currentZero = sum / averages;
}

//Measure the average angular velocity over <averages> samples at zero voltage (and thus zero 
//..current, zero torque and zero velocity), and store the result in currentZero. This value is then 
//..used to null out any offset in the measured velocity.
//Must be called with the PWM outputs on, but with the control loops disabled.
void velocityZeroCalibration(uint16_t averages)
{
    setPwm(0);
    velocityZero = 0;
    int32_t sum = 0;
    for(uint16_t n = 0; n < averages; ++n)
    {
        while (samplingState != LATEST_VELOCITY);
        sum -= velocity;
        while (samplingState == LATEST_VELOCITY);
    }
    velocityZero = sum / averages;
}

void angleZeroCalibration(uint16_t averages, int16_t outputLevel)
{
    //angleZero = 0;

    setPwm(-outputLevel);
    _delay_ms(100);

    int32_t sum_min = 0;
    for(uint16_t n = 0; n < averages; ++n)
    {
        while (samplingState != LATEST_ANGLE);
        sum_min += angle;
        while (samplingState == LATEST_ANGLE);
    }

    setPwm(outputLevel);
    _delay_ms(100);

    int32_t sum_max = 0;
    for(uint16_t n = 0; n < averages; ++n)
    {
        while (samplingState != LATEST_ANGLE);
        sum_max += angle;
        while (samplingState == LATEST_ANGLE);
    }

    setPwm(0);
    angleZero = 512; //(sum_max + sum_min) / (averages * 2);
    if(sum_max < sum_min) 
        coilInverted = 1;
}