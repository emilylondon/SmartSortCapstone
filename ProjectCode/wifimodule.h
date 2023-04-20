#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
void serial_init(void);
void serial_out(char);
void print_serial(char *);
char serial_in(void);
uint8_t read_line(char *);
void esp_init();
void tcp_init();
void tcp_send(char);
