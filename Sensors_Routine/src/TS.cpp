#include <Arduino.h>
#include "TS.h"

#define ts_pin 12

#define res 189
#define alpha 3.98e-3
#define Vref 5.0
#define Ro 100.0  

unsigned int ts_cnt = 0;
uint16_t ts_sum = 0;
float ts;

float temp;
float Rpt;

void configTS(){
    pinMode(ts_pin, INPUT);
}

float calibTS(float Vout){
    Vout *= 3.3/4096.0;
    Vout += .1;
    Rpt = (Vout*res)/(Vref-Vout); 
    temp = (Rpt-Ro)/(Ro*alpha);
    return temp;
}

float readTS(){
    ts = calibTS(ts_sum / ts_cnt);
    ts_sum = 0;
    ts_cnt = 0;
    return ts;
}

void checkTS(){
    ts_sum += analogRead(ts_pin);
    ts_cnt++;
}