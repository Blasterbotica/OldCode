/* This code is intended for the Arduino Uno built into the
 *  prototype controller for the drivetrain platform. It 
 *  uses analog input read from the vertical axis of each 
 *  joystick and converts it to a value that is written to 
 *  one of the two Roboclaws. The left joystick will control
 *  both the front and rear tires on the left side, and the
 *  right joystick will work in a similar manner for the 
 *  tires on the right side. PID variables for the Roboclaws
 *  can be changed in the calibration variables section.
 */

//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include "BMSerial.h"
#include "RoboClaw.h"
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <Servo.h>
#include <math.h>

// Roboclaw addresses
#define left_motor 0x80
#define right_motor 0x81

// Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0
#define qpps_max 904000.0// Quadrature pulses per second: 360 ticks/turn * 2 channels (quadrature) * 2 edges (1 rising, 1 falling) * max turns per second

// Current limit, in units of 10mA (current_limit of 100 --> 1A actual limit)
#define current_limit 1100 // 1100 --> 11A. This is approx 1/7 of 77A stall current, ~= max efficiency current for motor

// Variables assigned during loop()
float l_vert, r_vert;
int dpad_vert, dpad_horiz;
long int qpps_set_left = 0;
long int qpps_set_right = 0;
unsigned long last_msg_time;

//Setup communcations with roboclaw. Use pins 10 and 11 with 10ms timeout. Pin 10 goes to S2 on roboclaw; Pin 11 to S1
RoboClaw roboclaw(10,11,10000);
ros::NodeHandle nh;
Servo test_servo;


void joystick_cb( const sensor_msgs::Joy& joy_msg ){
  if (joy_msg.buttons[0] == 1) {          // A

  } else if (joy_msg.buttons[1] == 1) {   // B
    
  } else if (joy_msg.buttons[2] == 1) {   // X
    
  } else if (joy_msg.buttons[3] == 1) {   // Y
    
  } else if (joy_msg.buttons[4] == 1) {   // Left Bumper
    
  } else if (joy_msg.buttons[5] == 1) {   // Right Bumper
    
  }

  l_vert = (int) (joy_msg.axes[1] * 127.0);
  r_vert = (int) (joy_msg.axes[4] * 127.0);

  if (l_vert >= 0) {
    roboclaw.ForwardM1(left_motor, l_vert); // Output to motor
    roboclaw.ForwardM2(left_motor, l_vert); // Output to motor
  } else {
    roboclaw.BackwardM1(left_motor, -l_vert); // Output to motor
    roboclaw.BackwardM2(left_motor, -l_vert); // Output to motor
  }

  if (r_vert >= 0) {
    roboclaw.ForwardM1(right_motor, r_vert); // Output to motor
    roboclaw.ForwardM2(right_motor, r_vert); // Output to motor
  } else {
    roboclaw.BackwardM1(right_motor, -r_vert); // Output to motor
    roboclaw.BackwardM2(right_motor, -r_vert); // Output to motor
  }  
//  dpad_vert = joy_msg.axes[7];
//  dpad_horiz = joy_msg.axes[6];
//
//  if ((l_vert < 0.3) && (l_vert > -0.3)) {
//    l_vert = 0;
//  }
//  if ((r_vert < 0.3) && (r_vert > -0.3)) {
//    r_vert = 0;
//  }

//  qpps_set_left = qpps_max*l_vert; // Set percentage of max qpps
//  qpps_set_right = qpps_max*r_vert; // Set percentage of max qpps
//  
//  roboclaw.ForwardM1(left_motor, qpps_set_left); // Output to motor
//  roboclaw.ForwardM2(left_motor, qpps_set_left); // Output to motor
//
//  roboclaw.ForwardM1(right_motor, -qpps_set_right); // Output to motor
//  roboclaw.ForwardM2(right_motor, -qpps_set_right); // Output to motor 
  
//  test_servo.write(-atan2(joy_msg.axes[1], -joy_msg.axes[0])*180/3.14);
//  digitalWrite(13, HIGH);   // set the led high to provide local feedback that data has been sent
//  for(int i = 0; i < 50; i++) { // delay 500 ms
//    delayMicroseconds(10000);
//  }
//  digitalWrite(13, LOW);
}

ros::Subscriber<sensor_msgs::Joy> joystick_sub("joyslow", *joystick_cb);

void setup() {
  
  nh.initNode();
  nh.subscribe(joystick_sub);
  pinMode(13, OUTPUT);
  // Open roboclaw serial communication
  roboclaw.begin(9600);
//  Serial.begin(9600);
  
  // Set PID coefficients and addresses
  roboclaw.SetM1VelocityPID(left_motor,Kd,Kp,Ki,qpps_max);
  roboclaw.SetM2VelocityPID(left_motor,Kd,Kp,Ki,qpps_max);
  roboclaw.SetM1VelocityPID(right_motor,Kd,Kp,Ki,qpps_max);
  roboclaw.SetM2VelocityPID(right_motor,Kd,Kp,Ki,qpps_max);

  // Set hard current limits
  roboclaw.SetM1MaxCurrent(left_motor, current_limit);
  roboclaw.SetM2MaxCurrent(left_motor, current_limit);
  roboclaw.SetM1MaxCurrent(right_motor, current_limit);
  roboclaw.SetM2MaxCurrent(right_motor, current_limit);


//  test_servo.attach(10);

}

void loop() {

//  Serial.print("Left: ");
//  Serial.println(l_vert);
//  Serial.print("Right: ");
//  Serial.println(r_vert);
  

//  qpps_set_left = qpps_max*l_vert; // Set percentage of max qpps
//  qpps_set_right = qpps_max*r_vert; // Set percentage of max qpps
//  
//  roboclaw.SpeedM1(left_motor,-qpps_set_left); // Output to motor
//  roboclaw.SpeedM2(left_motor,-qpps_set_left); // Output to motor
//
//  roboclaw.SpeedM1(right_motor,qpps_set_right); // Output to motor
//  roboclaw.SpeedM2(right_motor,qpps_set_right); // Output to motor  

  nh.spinOnce();
}
