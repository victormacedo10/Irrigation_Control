#include <Arduino.h>

void IRAM_ATTR isrPluv(void* arg);
void enableInterruptP();
void configPluv();
uint16_t readPluv();
void checkPluv();
