#include <Arduino.h>
#include "rom/gpio.h"
#include "WS.h"

//semáforo para gerenciar o acesso às tarefas.
SemaphoreHandle_t xSemaphore_WS = xSemaphoreCreateBinary();
 
hw_timer_t * timer_ws = NULL;

//12,27,25,32
#define ESP_INTR_FLAG_DEFAULT 0
//O pino de GPIO tem um tipo específico, por isso deve-se utilizar o
//formato pré-definido ou então fazer o define passando a macro.
#define ws_pin GPIO_NUM_19
#define Ts_timer_ws 10 

byte ws_st = 0; // estado dos pinos são armazenados nesse byte

bool ws_int = false; //Usado na task da interrupção de alimentação
unsigned int ws_cnt = 0;
unsigned int ws_cnt_timer = 0;

//declaração do timer
void timerWS(){
    ws_cnt_timer++;
}

void startTimerWS(){
    timer_ws = timerBegin(1, 80, true);
    timerAttachInterrupt(timer_ws, &timerWS, true);
    timerAlarmWrite(timer_ws, Ts_timer_ws*1000, true); 
    //ativa o alarme
    timerAlarmEnable(timer_ws);
}

//declaração da ISR
void IRAM_ATTR isrWS(void* arg){
    xSemaphoreGiveFromISR(xSemaphore_WS, pdFALSE);
    ws_int = true;
}
 
//habilitando as interrupções    interrupted = false;
void enableInterruptWS(){
  //equivalente a pinMode() do Arduino
  gpio_set_pull_mode(ws_pin,GPIO_PULLUP_ONLY);
 
  //sem isso, as interrupções não funcionam
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
  //tipo da interrupção
  gpio_set_intr_type(ws_pin, GPIO_INTR_NEGEDGE);
 
  //habilita interrupção no pino X
  gpio_intr_enable(ws_pin);
 
  //manipulador
  gpio_isr_handler_add(ws_pin, isrWS, NULL);
}
 
void configWS(){
    //O mesmo para o pino de interrupção
    gpio_set_direction(ws_pin, GPIO_MODE_INPUT);
    enableInterruptWS();
    startTimerWS();
}
 
float readWS(){
    float f_ws = (ws_cnt/2.0) / (Ts_timer_ws/1000.0)*ws_cnt_timer; //f em ciclos/s
    float ws_out = (f_ws/1.2517) + 0.28;
    ws_cnt = 0;
    ws_cnt_timer = 0;
    return ws_out;
}

void checkWS(){
    if (ws_int){
        ws_st = digitalRead(ws_pin);
        if (ws_st == 0){
            ws_cnt++;
        }
        gpio_intr_enable(ws_pin);
        ws_int = false;
    }
}
