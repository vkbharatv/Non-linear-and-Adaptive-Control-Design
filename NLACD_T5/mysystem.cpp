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
  read_rmp();
  error = setpoint - _rpm;
  u = pid(error);
  move_motor(u);
}
void read_rmp()
{
  // RPM reading implementation
}
void motorSys::pid(double error)
{
  I += error * dt;
  D = (error - error_prev) / dt;
  P = error;
  u = Kp * P + Ki * I + Kd * D;
  u = constrain(u, u_min, u_max);
  error_prev = error;
}
void motorSys::set_pid_gains(double kp, double ki, double kd)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
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

void motorSys::enc_ISR()
{
  _encoder_count++;
}

void motorSys::setup_timer_interrupt(unsigned long dt_us, unsigned int prescaler)
{
  _instance = this;
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  const unsigned long tick_hz = 16000000UL / prescaler;
  const float dt_seconds = dt_us / 1000000.0f;
  unsigned long ocr_value = (unsigned long)(tick_hz * dt_seconds) - 1;

  if (ocr_value > 65535)
    ocr_value = 65535;
  OCR1A = (uint16_t)ocr_value;

  TCCR1B |= (1 << WGM12);

  switch (prescaler)
  {
  case 1:
    TCCR1B |= (1 << CS10);
    break;
  case 8:
    TCCR1B |= (1 << CS11);
    break;
  case 64:
    TCCR1B |= (1 << CS11) | (1 << CS10);
    break;
  case 256:
    TCCR1B |= (1 << CS12);
    break;
  case 1024:
    TCCR1B |= (1 << CS12) | (1 << CS10);
    break;
  default:
    TCCR1B |= (1 << CS12);
    break;
  }

  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}
