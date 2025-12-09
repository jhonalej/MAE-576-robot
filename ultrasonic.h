#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <avr/io.h>
#include <stdint.h>


// this contains everything need to make the ultrasonic sensor work by holding the pointers to trig and echo pins and port.
// this allows us to use any pins we want. 
typedef struct {
    volatile uint8_t *trig_port; 
    volatile uint8_t *trig_ddr;
    uint8_t trig_bit;

    volatile uint8_t *echo_pin;
    volatile uint8_t *echo_ddr;
    uint8_t echo_bit;
} Ultrasonic_t;

void Ultrasonic_init(Ultrasonic_t *us);
uint16_t Ultrasonic_measure(Ultrasonic_t *us);

#endif
