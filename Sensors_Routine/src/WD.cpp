#include <Arduino.h>
#include "WD.h"

#define wd_pin 13

unsigned int wd_cnt = 0;
uint16_t wd_sum = 0;
uint16_t wd;

void configWD(){
    pinMode(wd_pin, INPUT);
}

uint16_t calibWD(uint16_t x){
    uint16_t y = x/1000;
    return y;
}

float readWD(){
    wd = calibWD(wd_sum / wd_cnt);
    wd_sum = 0;
    wd_cnt = 0;
    return wd;
}

void checkWD(){
    wd_sum += analogRead(wd_pin);
    wd_cnt++;
}