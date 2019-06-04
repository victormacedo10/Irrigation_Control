#include <Arduino.h>

int soil1 = 32, soil2 = 25, soil3 = 34, soil4 = 35;
int window = 30;

void setup() {
  Serial.begin(115200);
}

int readValues(int pin1){
  soil1 = analogRead(pin1);
  //soil2 = analogRead(pin2n);
  //soil3 = analogRead(pin3);
  //soil4 = analogRead(pin4);
  return soil1;
}

void loop() {
  // put your main code here, to run repeatedly:
  int x = 0;
  for(int i=0; i<window; i++){
    x += readValues(soil2);
  }
  x /= window;
  Serial.println(x);
  delay(50);
}