#include "ultrasonic.h"
#include <util/delay.h>


// here im initilizing the inputs and and outputs of the trig and echo using pointers
void Ultrasonic_init(Ultrasonic_t *us)
{
    *(us->trig_ddr) |= (1 << us->trig_bit);     // TRIG output
    *(us->echo_ddr) &= ~(1 << us->echo_bit);    // ECHO input
}


//
uint16_t Ultrasonic_measure(Ultrasonic_t *us)
{
    uint16_t count = 0;
    uint32_t timeout = 0;

    // This code section im sending out pulses to the trig to be able to emit the sound wave
    *(us->trig_port) &= ~(1 << us->trig_bit);
    _delay_us(2);
    *(us->trig_port) |=  (1 << us->trig_bit);
    _delay_us(10);
    *(us->trig_port) &= ~(1 << us->trig_bit);

    // in here we wait till echo pickes up the wave meaing it goes high if it never does it returns a 0 meaing no wave detection
    while (!(*(us->echo_pin) & (1 << us->echo_bit))) {
        if (++timeout > 60000) return 0;
    }

    // we use timer1 to measure the amount of time that echo is high for, allowing for accurate measures.
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TCNT1 = 0;

    // we count how long the it was high by looking at the count of timer ticks
    timeout = 0;
    while (*(us->echo_pin) & (1 << us->echo_bit)) {
        if (++timeout > 60000) break;
    }

    TCCR1B = 0;
    count = TCNT1;
    // converting it to miroseconds and from that calulate the distance in CM 
    float time_us = count * 0.5f;
    return (uint16_t)(time_us / 58.0f);
}
