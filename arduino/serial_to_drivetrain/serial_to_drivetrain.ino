/*
 * Code for all drivetrain-related controls.
 * 
 * Must be written to as follows:
 *    Serial.write(left value);
 *    Serial.write(right value);
 * 
 * The left and right values will be in the range [0..255]
 * 
 */

#include "BMSerial.h"
#include "RoboClaw.h"
#include <math.h>

// Roboclaw addresses
#define left_motor 0x80
#define right_motor 0x81

// Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps_max 42400.0 // Quadrature pulses per second: 360 ticks/turn * 2 channels (quadrature) * 2 edges (1 rising, 1 falling) * max turns per second

// Current limit, in units of 10mA (current_limit of 100 --> 1A actual limit)
#define current_limit 2000 // 2000 --> 20A. This is approx 1/7 of 77A stall current, ~= max efficiency current for motor

// Variables assigned during loop()
int16_t currentr1; // Roboclaw sends current values as a 16 bit character
int16_t currentr2; //Right rear
int16_t currentl1; //Left front
int16_t currentl2; //Left rear
float l_vert, r_vert;
int dpad_vert, dpad_horiz;
long int qpps_set_left = 0;
long int qpps_set_right = 0;
unsigned long last_msg_time;
bool ledState = 0;
bool disc = true;
unsigned long msgtime = 0;
bool cooldown = false;

int lvert = 0;
int rvert = 0;

//Setup communcations with roboclaw. Use pins 10 and 11 with 10ms timeout. Pin 10 goes to S2 on roboclaw; Pin 11 to S1
RoboClaw roboclaw(10, 11, 10000);

int format_to_pct(int received_value) {
  return received_value;
}

void setup() {
  Serial.begin(115200);
  roboclaw.begin(9600);
  pinMode(13,OUTPUT);

//  // Set PID coefficients and addresses
//  roboclaw.SetM1VelocityPID(left_motor,Kd,Kp,Ki,qpps_max);
//  roboclaw.SetM2VelocityPID(left_motor,Kd,Kp,Ki,qpps_max);
//  roboclaw.SetM1VelocityPID(right_motor,Kd,Kp,Ki,qpps_max);
//  roboclaw.SetM2VelocityPID(right_motor,Kd,Kp,Ki,qpps_max);
//
  // Set hard current limits
//  roboclaw.SetM1MaxCurrent(left_motor, current_limit);
//  roboclaw.SetM2MaxCurrent(left_motor, current_limit);
//  roboclaw.SetM1MaxCurrent(right_motor, current_limit);
//  roboclaw.SetM2MaxCurrent(right_motor, current_limit);
}

void loop() {
  if(Serial.available() > 3) {
    while(Serial.available()>3){
      Serial.read();
    } 
    disc = false;
    msgtime = millis();

    byte msgl = Serial.read();
    byte msgr;
    if (!((msgl & 0x80) == 0x80)) {
      msgl = Serial.read();
    } 

    if((msgl & 0x80) == 0x80) {
      msgr = Serial.read();
      if((msgr & 0x80) != 0x80) {
        //char stat[80];
        l_vert = 2*int(msgl & 0x7F) - 128;
        r_vert = 2*int(msgr & 0x7F) - 128;
        roboclaw.ReadCurrents(right_motor, currentr1, currentr2);
        roboclaw.ReadCurrents(left_motor, currentl1, currentl2);
        char buffer[50];
        sprintf(buffer, "%d,%d,%d,%d\n", currentr1*10, currentr2*10, currentl1*10, currentl2*10);
        Serial.write(buffer);
      } else {
        Serial.write("ERR2\n");
      } 
    } else {
      Serial.write("ERR1\n");
    }
    
    // Toggle led on message receipt
    if (l_vert == 0 && r_vert == 0) {
      digitalWrite(13, LOW);
    } else {
      digitalWrite(13, HIGH);
    }
  } else {
    if ((millis()-msgtime) > 250) {
      disc = true;
    }
  }

  if (disc) {
    l_vert = 0;
    r_vert = 0;
  }

  if (!cooldown) {
    if (l_vert >= 0) {
      roboclaw.BackwardM1(left_motor, l_vert); // Output to motor
      roboclaw.BackwardM2(left_motor, l_vert); // Output to motor
    } else {
      roboclaw.ForwardM1(left_motor, -l_vert); // Output to motor
      roboclaw.ForwardM2(left_motor, -l_vert); // Output to motor
    }
  
    if (r_vert >= 0) {
      roboclaw.ForwardM1(right_motor, r_vert); // Output to motor
      roboclaw.ForwardM2(right_motor, r_vert); // Output to motor
    } else {
      roboclaw.BackwardM1(right_motor, -r_vert); // Output to motor
      roboclaw.BackwardM2(right_motor, -r_vert); // Output to motor
    }  
    
  //    qpps_set_left = qpps_max*format_to_pct(left_received); // Set percentage of max qpps
  //    qpps_set_right = qpps_max*format_to_pct(right_received); // Set percentage of max qpps
  //    
  //    roboclaw.SpeedM1(left_motor, qpps_set_left); // Output to motor
  //    roboclaw.SpeedM2(left_motor, qpps_set_left); // Output to motor
  //  
  //    roboclaw.SpeedM1(right_motor, -qpps_set_right); // Output to motor
  //    roboclaw.SpeedM2(right_motor, -qpps_set_right); // Output to motor 
  }
}
