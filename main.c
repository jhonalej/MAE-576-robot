/*Jhon Ramirez
this code is to make a 4WD robot move using an ultrasonic 
sensor to see in front and a gyroscope to detect when its 
being picked up. this is a project for my MAE 576 class, and 
will be build up for a future bigger project */

/*the included libaries need to make the project work with the main library being AVR.
this library is to allow us to use code the atmega328p using register level coding. 
the second library allows us to use delays, stdbool lets us use true or false in our 
code (use when reading from the mpu). lastly the math library allows us to use mathmatical functions
*/

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <math.h>
//this is a wire library found online for C programing 
#include "i2cmaster.h"

/*these are libraries I made to minimize the amount of lines of code that are in the main code  */
#include "UART.h"
#include "ultrasonic.h"

/*this following section is to define all of the pins incase another board is used or if pin get moved around
this makes it easier to change see the what im controlling*/

//this is pin definition for the ultrasonic, where the trigger is Pin D13 on the aruduino and the echo is D2 
#define TRIG_PORT  PORTB
#define TRIG_DDR   DDRB
#define TRIG_BIT   PB5   

#define ECHO_PIN   PIND
#define ECHO_DDR   DDRD
#define ECHO_BIT   PD2   

//this is pin defenition for the reset button A1 
#define RESET_BIT   PC1
#define RESET_PIN   PINC
#define RESET_DDR   DDRC
#define RESET_PORT  PORTC

extern uint8_t current_mode;

//indicater led pin define it is connected to Pin D10 on the arduino
#define FAULT_LED_BIT  PB2   
#define FAULT_LED_DDR  DDRB
#define FAULT_LED_PORT PORTB

//defining pins for the mode buttons Pin A2 and A0 
#define MODE_FOLLOW_BIT PC0
#define MODE_AVOID_BIT  PC2

uint8_t current_mode = 0;  
// 0 = none
// 1 = follow
// 2 = avoid

/*motor pins, since a motor needs a PWM to control the speed each motor 
has its own direction pin (not PWM ) and a PWM pin */
//the follow pins are as follow 

//left front motor Pin D7 and D5(pwm)
#define LF_DIR   PD7
#define LF_PWM   PD5

//left back motor Pins D4 and D6(pwm)
#define LB_DIR   PD4
#define LB_PWM   PD6

//right front motor Pin D8 and D3(pwm)
#define RF_DIR   PB4
#define RF_PWM   PB3

//right back motor Pin D12 and D11 (pwm)
#define RB_DIR   PB0
#define RB_PWM   PD3

/*these defenitons are use to make it easier to tune the PWM so that
 we can make it faster or slower 0 being slow and 255 being the fast*/

#define FULL_SPEED   200
#define MID_SPEED    125
#define SLOW_SPEED    80
#define TURN_SPEED    60

//this function allows me to use PWM 
void pwm_init(void)
{
    // start by setting the pins to be outputs for the pins D5, D6, D3 and D11
    DDRD |= (1<<PD5) | (1<<PD6);
    DDRB |= (1<<PB3);
    DDRD |= (1<<PD3);

    //here we are using timer0 to control, here we control which PWM mode we are going to us
    //in this case I use the fast PWM mode 8-bit(from datasheet)
    TCCR0A = (1<<WGM00)|(1<<WGM01)
           | (1<<COM0A1)|(1<<COM0B1);// i use the none inverting pin so that it outputs hight at first and low when it reaches the set value
    TCCR0B = (1<<CS01);

    // the same process is done for Timer2 only for pins D3 and D11
    TCCR2A = (1<<WGM20)|(1<<WGM21)
           | (1<<COM2A1)|(1<<COM2B1);
    TCCR2B = (1<<CS21);
}

/*this set of code is now to make the motors move using function. 
These functions include motor initilize, setting the speed, making 
the the motors move forward, turning left and stoping completly. */

// this is just to configure the pins as outputs
void motors_init(void)
{
    DDRD |= (1<<LF_DIR) | (1<<LB_DIR);
    DDRB |= (1<<RF_DIR) | (1<<RB_DIR);
}
//this code is  use to control the PWM from above by changing the values to a desired number
void motors_set_speed(uint8_t s)
{
    OCR0A = s;
    OCR0B = s;
    OCR2A = s;
    OCR2B = s;
}

//this moves the motors forward by setting all the direction pins to be low, 
//this moves the robot forward. this of course depends how you wire the motor leads
void motors_forward(void)
{
    PORTD &= ~(1<<LF_DIR);
    PORTD &= ~(1<<LB_DIR);
    PORTB &= ~(1<<RF_DIR);
    PORTB &= ~(1<<RB_DIR);

    motors_set_speed(FULL_SPEED);
}

// this makes the robot move left by making the left motors on high and the right low. 
// This makes the left motor moves backwards and right move forward 
void motors_left_turn(void)
{
    PORTD |=  (1<<LF_DIR);
    PORTD |=  (1<<LB_DIR);

    PORTB &= ~(1<<RF_DIR);
    PORTB &= ~(1<<RB_DIR);

    motors_set_speed(TURN_SPEED);
}

// this just makes the PWM be 0 so nothing moves
void motors_stop(void)
{
    motors_set_speed(0);
}

/*to better know what is going on in the robot's "brain" an 
LED is uses to know when a mode is selected and when there is a fault. 
this is done by using a for loop to make it blink how ever many times. 
in this case either once or twice depending on mode  */
void fault_led_blink(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        FAULT_LED_PORT |= (1<<FAULT_LED_BIT);
        _delay_ms(200);
        FAULT_LED_PORT &= ~(1<<FAULT_LED_BIT);
        _delay_ms(200);
    }
}


/*to be able to restart the robot a reset button is need. 
The following code takes care of the logic for it all*/
void wait_for_reset_button(void)
{
    UART_tx_string("NEED RESET BUTTON (A1)\r\n");
    //the arduino stays in this while loop till as long as the button is not pressed. 
    while (RESET_PIN & (1<<RESET_BIT))
        ;

    _delay_ms(50);
    // this does the same thing except that when the button is press it to continue with the rest of the code
    while (!(RESET_PIN & (1<<RESET_BIT)))
        ;

    UART_tx_string("RESET PRESSED — RESUMING\r\n");
    // returns to the default mode which is 0 meaing it will wait for an input from the mode buttons and turns LED off 
    current_mode = 0;
    FAULT_LED_PORT &= ~(1<<FAULT_LED_BIT); 

    UART_tx_string("MODE CLEARED — TAP A0 (FOLLOW) OR A2 (AVOID)\r\n");

    _delay_ms(150);
}


/* The function below are use to make the MPU (gyroscope work). to use the MPU we use the i2c library 
to be able to communicate with the arduino. as well as some math to be able to use the raw data to be useble data*/

// start by making the defenition of the MPU addess so that i2c knows who to read and write
#define MPU_ADDR 0x68

//this is function allows us to write to the MPU to be able to initilize it by first sending the start conditions.
void mpu_write(uint8_t reg, uint8_t val)
{
    if (i2c_start((MPU_ADDR<<1)|I2C_WRITE)) { i2c_stop(); return; }
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

// this functions allows us to turn on the MPU from sleeping by 
//sending the 0x00 to the 0x6B which is the power maneger address 
//of the MPU. lastly we make it so that the gyrsocope (0x1B uses full sensativity range)
void mpu_init(void)
{
    mpu_write(0x6B, 0x00);
    mpu_write(0x1B, 0x00);
}

// this is the bulk of the MPU where here we get the data needed to detect if the robot is being picked up. 
bool mpu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    // this tell the MPU which register you want to read 
    if (i2c_start((MPU_ADDR<<1)|I2C_WRITE)) return false;
    i2c_write(0x43);
    if (i2c_rep_start((MPU_ADDR<<1)|I2C_READ)) return false;

    // using the i2c library we tell the MPU to read the high bytes then the low bytes to get the data that. 
    uint8_t xh=i2c_readAck(), xl=i2c_readAck();
    uint8_t yh=i2c_readAck(), yl=i2c_readAck();
    uint8_t zh=i2c_readAck(), zl=i2c_readNak();
    i2c_stop();

    //since we are only able to send 8 bits at a time we need to combine the data back to 16 bits to be able to use for detection
    *gx = (xh<<8)|xl;
    *gy = (yh<<8)|yl;
    *gz = (zh<<8)|zl;
    return true;
}

// this function is used to create a median for the measured distance of dist so that if it spikes in number it wont break the code
uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    // If a is between b and c → a is median
    if ((a >= b && a <= c) || (a >= c && a <= b))
        return a;

    // If b is between a and c → b is median
    if ((b >= a && b <= c) || (b >= c && b <= a))
        return b;

    // Otherwise c is median
    return c;
}

// MAIN CODE
int main(void)
{
    // we start by initilizing all the needed components like the UART baud rate (from own library) i2c and MPU.
    UART_init(BAUD_9600);
    i2c_init();
    mpu_init();


    // we also set the buttons all to be inputs to read it to start, as well as 
    //set up the internal pull up resistor, since the buttons are active low. 
    RESET_DDR  &= ~(1<<RESET_BIT);
    RESET_PORT |=  (1<<RESET_BIT);

    DDRC &= ~((1<<MODE_FOLLOW_BIT) | (1<<MODE_AVOID_BIT));
    PORTC |=  (1<<MODE_FOLLOW_BIT) | (1<<MODE_AVOID_BIT);

    // Fault set up for the LED to be output and be low so its off
    FAULT_LED_DDR |= (1<<FAULT_LED_BIT);
    FAULT_LED_PORT &= ~(1<<FAULT_LED_BIT);   


    //this part of the code i am making an object to be able to send to the ultrasonic library init.
    // in this object im including the port, address, and bit for both the echo and trig. 
    Ultrasonic_t us = {
        &TRIG_PORT, &TRIG_DDR, TRIG_BIT,
        &ECHO_PIN,  &ECHO_DDR, ECHO_BIT
    };
    //with the object created it gets pushed the address to init of the ultrasonic lib. this way any pins can be used for the ultra. 
    Ultrasonic_init(&us);

    // these lines of code are just used to also initilize the PWM as well as the motors and start the motors being on stop meaning not moving 
    pwm_init();
    motors_init();
    motors_stop();

    UART_tx_string("SYSTEM READY — TAP A0 to follow or A2 to avoid \r\n");


    //the bulk of the code
    while (1)
    {
        // when the current mode is 0 it makes it so that the robot is idle and will only be able to go back to 0 when the reset is pressed
        if (current_mode == 0)
        {
            // here we check is we are pressing the follow button, it will detect when its press and should be look twice to confirm it
            //it wasnt accidently press. 
            if (!(PINC & (1<<MODE_FOLLOW_BIT)))
            {
                _delay_ms(50);
                if (!(PINC & (1<<MODE_FOLLOW_BIT)))
                {
                    // here we change the varibale current_mode to 1 so the robot knows to follow as well as make the LED blink onces 
                    current_mode = 1;
                    UART_tx_string("MODE FOLLOW\r\n");

                    fault_led_blink(1);  // 1 BLINK
                    // this makes it so that it knows when its being relesed ensuring it actives onces. 
                    while (!(PINC & (1<<MODE_FOLLOW_BIT)));
                    _delay_ms(50);
                }
            }

            // Avoid selection
            //same process as follow mode. 
            if (!(PINC & (1<<MODE_AVOID_BIT)))
            {
                _delay_ms(50);
                if (!(PINC & (1<<MODE_AVOID_BIT)))
                {
                    current_mode = 2;
                    UART_tx_string("MODE AVOID\r\n");

                    fault_led_blink(2);  // 2 BLINKS

                    while (!(PINC & (1<<MODE_AVOID_BIT)));
                    _delay_ms(50);
                }
            }
        }
        
        // here we first look to see what mode we are in and depending on the mode the logic will be preformed. 
        if (current_mode == 0)
        {
            //Mode = 0 means its idle 
            motors_stop();
            UART_tx_string("NO MODE TAP A0 OR A2\n");
            _delay_ms(150);
            continue;
        }


        // in this section we first create varibles to be able to use in the follow and fault detection.
        uint16_t d1 = Ultrasonic_measure(&us);
        _delay_ms(10);
        uint16_t d2 = Ultrasonic_measure(&us);
        _delay_ms(10);
        uint16_t d3 = Ultrasonic_measure(&us);

        uint16_t dist = median3(d1, d2, d3);

        int16_t gx, gy, gz;
        float tilt = 0;

        // here we used the function read_gyro from above to get the values from raw data to degrees/sec.
        if (mpu_read_gyro(&gx,&gy,&gz))
            // since we care more about it being picked up or rotated we look at the absolute values of Gy and Gz, and convert them to useable data
            tilt = fabsf(gy/131.0f) + fabsf(gz/131.0f);

        // based on the calulation from above we check to see if it passes a set value which in this case is 80. meaning if tilt is greate than 80.
        //it indicates that it got picked up and will stop the robot, turn LED on and call teh rest button fuction.
        if (tilt > 80)
        {
            FAULT_LED_PORT |= (1<<FAULT_LED_BIT);  
            motors_stop();
            wait_for_reset_button();
            continue;
        }

        // this is the section when the robot is in FOLLOW mode.
        // here it will look at the distance and depending on how far and object is it will stop or go.
        if (current_mode == 1)
        {
            UART_tx_string("[FOLLOW]\r\n");

            if (dist <= 20)
                motors_stop();
            else if (dist <= 60)
                motors_set_speed(FULL_SPEED);
            else
                motors_stop();
        }

        // this section is for the AVOID. meaning that the robot will avoid objects or walls in this case. if the ULT. reads less than 30
        // it will stop turn left and then move forward. it will also slow down when it starts approching a wall. and if nothing is there 
        //it will keep going forward full speed 
        else if (current_mode == 2)
        {
            UART_tx_string("[AVOID]\r\n");

            if (dist <= 10 && dist <=40)
            {
                motors_stop();
                _delay_ms(100);

                motors_left_turn();
                _delay_ms(700);

                motors_forward();
            }
            else if (dist <= 45)
                motors_set_speed(MID_SPEED);
            else
                motors_set_speed(FULL_SPEED);
        }

        _delay_ms(40);
    }
}
