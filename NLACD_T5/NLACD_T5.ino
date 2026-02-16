#include<Arduino.h>
#include"mysystem.h"

// declaration
// methods

motorSys sys1(5);

#define M1 5
#define M2 6
#define ENA 2
#define ENB 3


void setup(){
  sys1.SerialIni(9600);
  sys1.initializeport(M1, M2, ENA, ENB);
  sys1.publish_data();
  sys1.setpoint = 100;                 // Desired RPM
  sys1.set_pid_gains(1.0, 0.5, 0.1);   // Example PID gains
  sys1.setup_timer_interrupt(100, 64); //
}

void loop(){
}

void enc_ISR(){
  // Encoder interrupt service routine implementation
}