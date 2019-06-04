#include <Arduino.h>
#include "rom/gpio.h"
#include "Pluv.h"
 
//semáforo para gerenciar o acesso às tarefas.
SemaphoreHandle_t xSemaphore_P = xSemaphoreCreateBinary();

//12,27,25,32
#define ESP_INTR_FLAG_DEFAULT 0
//O pino de GPIO tem um tipo específico, por isso deve-se utilizar o
//formato pré-definido ou então fazer o define passando a macro.
#define pluv_pin GPIO_NUM_19
 
byte pluv_st = 0; // estado dos pinos são armazenados nesse byte

bool pluv_int = false; //Usado na task da interrupção de alimentação
unsigned int pluv_cnt = 0;

//declaração da ISR
void IRAM_ATTR isrPluv(void* arg){
    xSemaphoreGiveFromISR(xSemaphore_P, pdFALSE);
    pluv_int = true;
}
 
//habilitando as interrupções    interrupted = false;
void enableInterruptP(){
  //equivalente a pinMode() do Arduino
  gpio_set_pull_mode(pluv_pin,GPIO_PULLUP_ONLY);
 
  //sem isso, as interrupções não funcionam
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
  //tipo da interrupção
  gpio_set_intr_type(pluv_pin, GPIO_INTR_NEGEDGE);
 
  //habilita interrupção no pino X
  gpio_intr_enable(pluv_pin);
 
  //manipulador
  gpio_isr_handler_add(pluv_pin, isrPluv, NULL);
}
 
void configPluv(){
    //O mesmo para o pino de interrupção
    gpio_set_direction(pluv_pin, GPIO_MODE_INPUT);
    enableInterruptP();
}
 
uint16_t readPluv(){
    uint16_t P_out = pluv_cnt*2;
    pluv_cnt = 0;
    return P_out;
}

void checkPluv(){
    if (pluv_int){
        pluv_st = digitalRead(pluv_pin);
        if (pluv_st == 0){
            pluv_cnt++;
        }
        gpio_intr_enable(pluv_pin);
        pluv_int = false;
    }
}
