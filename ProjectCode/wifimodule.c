#include "wifimodule.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#define FOSC 9830400
#define BAUD 9600
#define UBRR_SET (FOSC/16/BAUD-1) 

char buf[50];

// UBRR = oscfreq/(16*baud) - 1
void serial_init () {
    UBRR0 = UBRR_SET; // Set baud rate
    UCSR0B |= (1 << TXEN0 ); // Turn on transmitter
    UCSR0B |= (1 << RXEN0 ); // Turn on receiver
    UCSR0C = (3 << UCSZ00 ); // Set for async . operation , no parity ,
    // one stop bit , 8 data bits
 }

void serial_out ( char ch )
{
    while (( UCSR0A & (1 << UDRE0)) == 0);
    UDR0 = ch ;
}

void print_serial(char * line){
   while (*line > 0) serial_out(*line++);
}

char serial_in ()
{
    while ( !( UCSR0A & (1 << RXC0)) );
    return UDR0 ;
}

uint8_t read_line(char * buf){
    uint8_t ind = 0;
    char c;
    c = serial_in();
    while(c != '\n'){
        buf[ind] = c;
        ind++;
        c = serial_in();
    }
    buf[ind] = c;
    buf[ind+1] = 0;
    while((UCSR0A & (1 << RXC0))){
        c = UDR0;
    }
    return ind+1;
}

void esp_init(){
    serial_init();
    _delay_ms(10);
    snprintf(buf, 50, "AT+RST\r\n"); //Reset the ESP01 on startup
    print_serial(buf);
    _delay_ms(10);

    snprintf(buf, 50, "AT+CWMODE=1\r\n\0"); //Enable connections for the ESP01
    print_serial(buf);
    _delay_ms(10);

    snprintf(buf, 50, "AT+CWJAP=\"ssid\",\"password\"\r\n\0"); //Connect to wifi network
    print_serial(buf);
    tcp_init();
}

void tcp_init(){
    snprintf(buf, 50, "AT+CIPSTART=\"TCP\",\"172.20.10.3\",5030\r\n\0");
    print_serial(buf);
    _delay_ms(20);
}
void tcp_send(char ch){
    snprintf(buf, 50, "AT+CIPSEND=1\r\n\0");
    print_serial(buf);
    snprintf(buf, 50, "%d\0", ch);
    print_serial(buf);
}
