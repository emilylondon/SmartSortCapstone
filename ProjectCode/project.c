#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include "light_ws2812.h"
#include "wifimodule.h"
#include "proximitysensor.h"

#define INITIAL 0
#define PREDICT 1
#define TRASH 2
#define RECYCLING 3
#define COMPOST 4 

#define TRASH_BUTTON PD4
#define RECYCLE_BUTTON PD7 
#define COMPOST_BUTTON PB0 

#define COMP_STEPS 200
#define REC_STEPS -200
#define STEP_PIN PD3
#define STEP_DIR_PIN PD2
#define SERVO_PIN PB1

#define TRIG_PIN PD5
#define ECHO_PIN PB2
#define CONVERT 0.003489176432292

#define SERVO_MIN 1000
#define SERVO_MAX 2000

//gonna need to do timer stuff and pwm stuff for this...sad
//OC1A/PC1NT1 DDRB |= (1<<PB1);
// OCR1A and OCR1B
//

struct cRGB led[1];

uint16_t micros() {
  return (uint16_t) (TCNT1 * 4.0 / 9.8304);
}

char distance_thresh(){
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);

    // Wait for echo pin to go high
    while ((PINB & (1 << ECHO_PIN)) == 0);
    // Measure the pulse width of the echo signal
    uint16_t start_time = micros();
    while ((PINB & (1 << ECHO_PIN)) != 0);
    uint16_t pulse_duration = micros() - start_time;
    // Calculate the distance in cm 
    float distance = pulse_duration * CONVERT;

    return distance < 2 ? 1:0;
}

void servo_move(){
    //drop servo 
    OCR1A = (SERVO_MAX * 10) / 16 - 1;
    // wait a second for trash to fall 
    _delay_ms(1000);
    OCR1A = (SERVO_MIN * 10) / 16 - 1;
    //reset to max position 
    
}
void light_up(uint8_t state){
    if (state == PREDICT){
        led[0].r=50;led[0].g=50;led[0].b=100;    // Write white to array
    }
    else if (state == TRASH){
        led[0].r=255;led[0].g=00;led[0].b=0;    // Write red to array
    }
    else if (state == RECYCLING){
        led[0].r=0;led[0].g=00;led[0].b=255;    // Write blue to array
    }
    else if (state == COMPOST){
        led[0].r=0;led[0].g=255;led[0].b=0;    // Write green to array
    }
    ws2812_setleds(led,1);
}

void light_off(){
    led[0].r=0;led[0].g=0;led[0].b=0;
    ws2812_setleds(led, 1);
}

void go_steps(int steps){
    if (steps < 0){
        PORTD &= ~(1<<STEP_DIR_PIN);
        steps = -steps;
    }
    else {
       PORTD |= 1<<STEP_DIR_PIN;
    }

    for(int x = 0; x < -steps; x++)
    {
        PORTD |= 1<<STEP_PIN;
        _delay_us(2000);
        PORTD &= ~(1<<STEP_PIN);
	    _delay_us(2000);
    }
}
char trash_pressed(){
    if ((PIND & (1<<TRASH_BUTTON)) == 0){
        _delay_ms(5);
        while ((PIND & (1<<TRASH_BUTTON)) == 0);
        _delay_ms(5);
        return 1;
    }
    else {
        return 0;
    }
}

char recycle_pressed(){
    if ((PIND & (1<<RECYCLE_BUTTON)) == 0){
        _delay_ms(5);
        while ((PIND & (1<<RECYCLE_BUTTON)) == 0);
        _delay_ms(5);
        return 1;
    }
    else {
        return 0;
    }
}

char compost_pressed(){
    if ((PINB & (1<<COMPOST_BUTTON)) == 0){
        _delay_ms(5);
        while ((PINB & (1<<COMPOST_BUTTON)) == 0);
        _delay_ms(5);
        return 1;
    }
    else {
        return 0;
    }
}

int main(){
    DDRD &= ~(1<<PD4);
    DDRD &= ~(1<<PD7);
    DDRB &= ~(1<<PB0);
    PORTD |= (1<<PD4) | (1<<PD7); //pull up resistor
    PORTB |= 1<<PB0; //pull up resistor
    DDRB |= (1 << PB1); // Set PB1 as output for servo
    
    //Set up UDR pins 
    DDRD |= (1 << TRIG_PIN);           
    DDRB &= ~(1 << ECHO_PIN);      
    PORTB |= (1 << ECHO_PIN);         
    // Set up Timer 1 for PWM generation for servo 
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999; //top value for the timer 
    serial_init();
    uint8_t state = INITIAL;
    char pred[20];
    uint8_t s_sz = 0;
    while(1){
        if (state == INITIAL){
            if (compost_pressed()){
                state = COMPOST;
            }
            else if (recycle_pressed()){
                state = RECYCLING;
            }
            else if (trash_pressed()){
                state = TRASH;
            }
            else if (distance_thresh()){
                print_serial("go\r\n");
                state = PREDICT;
            }
        
        }
        else if (state == PREDICT){
            light_up(state);
            if (( UCSR0A & (1 << RXC0))){
                s_sz = read_line(pred);
                _delay_ms(30);
                if (pred[0] == 'c'){
                    state = RECYCLING;
                }
                else if (pred[0] == 'r'){
                    state = COMPOST;
                }
                else {
                    state = TRASH;
                }
                memset(pred, 0, sizeof pred);
                light_off();
            }
            else if (compost_pressed()){
                state = COMPOST;
            }
            else if (recycle_pressed()){
                state = RECYCLING;
            }
            else if (trash_pressed()){
                state = TRASH;
            }
        }
        else if (state == TRASH){
            light_up(state);
            _delay_ms(500);
            servo_move();
            light_off(); 
            state = INITIAL;
        }
        else if (state == RECYCLING){
            light_up(state);
            _delay_ms(500);
            go_steps(1000);
            servo_move();
            go_steps(-1000);
            light_off();
            state = INITIAL;
        }
        else {
            light_up(state);
            _delay_ms(500);
            go_steps(-1000);
            servo_move();
            go_steps(1000);
            light_off();
            state = INITIAL;
        }
        _delay_ms(20);
    }
}