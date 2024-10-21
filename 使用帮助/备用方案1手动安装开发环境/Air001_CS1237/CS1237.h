#ifndef CS1237_h
#define CS1237_h

#include <Arduino.h>

#define SPEED_10 0b00000000
#define SPEED_40 0b00010000 
#define SPEED_640 0b00100000
#define SPEED_1280 0b00110000 

#define PGA_1 0b00000000
#define PGA_2 0b00000100
#define PGA_64 0b00001000
#define PGA_128 0b00001100

#define CHANNEL_A 0b00000000
#define CHANNEL_Temp 0b00000010

void CS1237_init(uint32_t sck, uint32_t dout);
int CS1237_configure(uint8_t gain, uint8_t speed, uint8_t channel);
int32_t CS1237_read(void);

#endif
