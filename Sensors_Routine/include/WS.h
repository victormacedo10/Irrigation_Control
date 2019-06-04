#include <Arduino.h>

void IRAM_ATTR isrWS(void* arg);
void enableInterruptWS();
void configWS();
float readWS();
void checkWS();
