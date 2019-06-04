#include <Arduino.h>
#include "rom/gpio.h"
#include "Pluv.h"
//12,27,25,32
#define ESP_INTR_FLAG_DEFAULT 0
//O pino de GPIO tem um tipo específico, por isso deve-se utilizar o
//formato pré-definido ou então fazer o define passando a macro.
#define PIN_TO_INT            GPIO_NUM_19
 
byte st = 0; // estado dos pinos são armazenados nesse byte

bool interrupted    = false; //Usado na task da interrupção de alimentação
unsigned int pluv_cnt = 0;
 
//semáforo para gerenciar o acesso às tarefas.
SemaphoreHandle_t xSemaphore    = xSemaphoreCreateBinary();
 
//declaração da ISR
void IRAM_ATTR my_isr_handler(void* arg){
    xSemaphoreGiveFromISR(xSemaphore, pdFALSE);
    interrupted = true;
}
 
//habilitando as interrupções    interrupted = false;
void enableInterrupt(){
  //equivalente a pinMode() do Arduino
  gpio_set_pull_mode(PIN_TO_INT,GPIO_PULLUP_ONLY);
 
  //sem isso, as interrupções não funcionam
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
  //tipo da interrupção
  gpio_set_intr_type(PIN_TO_INT, GPIO_INTR_NEGEDGE);
 
  //habilita interrupção no pino X
  gpio_intr_enable(PIN_TO_INT);
 
  //manipulador
  gpio_isr_handler_add(PIN_TO_INT, my_isr_handler, NULL);
}
 
void configPluv(){
    //O mesmo para o pino de interrupção
    gpio_set_direction(PIN_TO_INT, GPIO_MODE_INPUT);
    enableInterrupt();
}
 
uint16_t readPluv(){
    uint16_t P_out = pluv_cnt*2;
    pluv_cnt = 0;
    return P_out;
}

void checkPluv(){
    if (interrupted){
        st = digitalRead(PIN_TO_INT);
        if (st == 0){
            pluv_cnt++;
        }
        gpio_intr_enable(PIN_TO_INT);
        interrupted = false;
    }
    delay(10);
}