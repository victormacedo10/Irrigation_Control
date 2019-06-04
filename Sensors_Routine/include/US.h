#include <Arduino.h>

struct Soil_res{
    uint8_t val1, val2, val3;
};

void configUS();
Soil_res readUS();
void checkUS();