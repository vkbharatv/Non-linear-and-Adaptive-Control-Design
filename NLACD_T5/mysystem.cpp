#include <Arduino.h>
#include "mysystem.h"

motorSys::motorSys(int a){
    x = a;
    _instance = this; // Set the static instance pointer to the current object
}

int motorSys::getdata(){
  return x;
}
void motorSys::SerialIni(int baudrate){
  Serial.begin(baudrate);
}
void motorSys::initializeport(int m1, int m2, int ena, int enb){
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(ena, INPUT_PULLUP);
  pinMode(enb, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ena), enc_ISR_wrapper, RISING);
}
void motorSys::publish_data(){
  Serial.print("MotorSys Status: ");
  Serial.println("Connected");
}

void motorSys::control_loop(){
  // Control loop implementation
}

double motorSys::get_rpm(){
  // RPM calculation implementation
}

motorSys* motorSys::_instance = nullptr;
void motorSys::enc_ISR_wrapper(){
  if (_instance != nullptr) {
    _instance->enc_ISR();
  }
}

void motorSys::enc_ISR(){
  // Encoder interrupt service routine
  _encoder_count++;
}
