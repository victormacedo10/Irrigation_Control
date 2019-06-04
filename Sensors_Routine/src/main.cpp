#include <Arduino.h>
#include "Pluv.h"
#include "US.h"
#include "TS.h"
#include "WD.h"
#include "WS.h"

hw_timer_t * timer = NULL;

// periodos em ms
uint16_t US_Ts = 100;
uint16_t TS_Ts = 100;
uint16_t BME_Ts = 2000;
uint16_t R_Ts = 100;
uint16_t WS_Ts = 10;
uint16_t WD_Ts = 2000;
uint16_t P_Ts = 10;
uint16_t TX_Ts = 5000;
uint16_t Ts[8] = {US_Ts, TS_Ts, BME_Ts, R_Ts, WS_Ts, WD_Ts, P_Ts, TX_Ts};
uint16_t min_Ts;

// variaveis para armazenar medidas
Soil_res US;
float TS;
float BME_U;
float BME_T;
float BME_P;
float R;
float WS;
uint16_t WD;
uint16_t P;

unsigned int read_cnt = 0;
bool timer_flag = false;

void cb_timer(){
    checkWS();
    checkPluv();
    read_cnt++;
    timer_flag = true;
}

void startTimer(){
    min_Ts = US_Ts;
    for (int i = 0; i < 8; i++){
        if (Ts[i] < min_Ts){
           min_Ts = Ts[i];
        }
    }
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &cb_timer, true);
    timerAlarmWrite(timer, min_Ts*1000, true); 
    //ativa o alarme
    timerAlarmEnable(timer);
}

void routineLoop(){
    if(timer_flag){
        if((read_cnt % ((int) (US_Ts/min_Ts))) == 0){
          checkUS();
        }
        if((read_cnt % ((int) (TS_Ts/min_Ts))) == 0){
          checkTS();
        }
        if((read_cnt % ((int) (BME_Ts/min_Ts))) == 0){
          Serial.print("BME ");
        }
        if((read_cnt % ((int) (R_Ts/min_Ts))) == 0){
          Serial.print("R ");
        }
        if((read_cnt % ((int) (WD_Ts/min_Ts))) == 0){
          checkWD();
        }
        if((read_cnt % ((int) (TX_Ts/min_Ts))) == 0){
          US = readUS();
          TS = readTS();
          WD = readWD();
          P = readPluv();
          WS = readWS();
          Serial.println();
          Serial.print("Enviando os dados");
          read_cnt = 0;
        }
        Serial.println();
        timer_flag = false;
    }
}

void configSensors(){
    configPluv();
    configUS();
    configTS();
    configWD();
    startTimer();
}

void setup(){
    Serial.begin(115200);
    Serial.println("Starting");
    configSensors();
}

void loop(){
    routineLoop();
}
