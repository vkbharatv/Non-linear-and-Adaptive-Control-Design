#include<Arduino.h>
#include"mysystem.h"

motorSys sys1(5);

#define M1 5
#define M2 6
#define ENA 2
#define ENB 3


void setup(){
  sys1.SerialIni(9600);
  sys1.initializeport(M1, M2, ENA, ENB);
  sys1.publish_data();
}

void loop(){
}

void enc_ISR(){
  // Encoder interrupt service routine implementation
}