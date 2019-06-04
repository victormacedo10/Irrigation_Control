#include <Arduino.h>
#include "US.h"

unsigned int us_cnt = 0;

struct Soil_config{
    uint8_t pin1, pin2, pin3;
    uint8_t sum1, sum2, sum3;
};

Soil_config soil {25, 34, 35, 0, 0, 0};

void configUS(){
    pinMode(soil.pin1, INPUT);
    pinMode(soil.pin2, INPUT);
    pinMode(soil.pin3, INPUT);
}
 
uint8_t calibUS(uint16_t x){
    uint8_t y = x/1000;
    return y;
}

Soil_res readUS(){

    Soil_res us;

    us.val1 = calibUS((uint16_t) soil.sum1 / us_cnt);
    us.val2 = calibUS((uint16_t) soil.sum2 / us_cnt);
    us.val3 = calibUS((uint16_t) soil.sum3 / us_cnt);
    soil.sum1 = 0;
    soil.sum2 = 0;
    soil.sum3 = 0;
    us_cnt = 0;
    return us;
}

void checkUS(){
    soil.sum1 += analogRead(soil.pin1);
    soil.sum2 += analogRead(soil.pin2);
    soil.sum3 += analogRead(soil.pin3);
    us_cnt++;
}

