#include <Arduino.h>
#include "Pluv.h"

hw_timer_t * timer = NULL;

// periodos em ms
uint16_t P_Ts = 5000;

// variaveis para armazenar medidas
uint16_t P;

void cb_timer(){
    static unsigned int read_cnt = 1;
    P = readPluv();
    Serial.print("P: ");
    Serial.println(P);
    read_cnt++;
}

void startTimer(){
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &cb_timer, true);
    timerAlarmWrite(timer, P_Ts*1000, true); 
    //ativa o alarme
    timerAlarmEnable(timer);
}

void setup(){
    configPluv();
    startTimer();
    Serial.begin(115200);
    Serial.println("Starting");
}

void loop(){
    checkPluv();
    delay(10);
}