#include <Arduino.h>

void IRAM_ATTR my_isr_handler(void* arg);
void enableInterrupt();
void configPluv();
uint16_t readPluv();
void checkPluv();