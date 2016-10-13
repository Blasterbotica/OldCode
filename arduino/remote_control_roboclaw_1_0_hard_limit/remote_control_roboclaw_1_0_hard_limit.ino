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


// Roboclaw addresses
#define left_motor 0x80
#define right_motor 0x81


// Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps_max 42400 // Quadrature pulses per second: 360 ticks/turn * 2 channels (quadrature) * 2 edges (1 rising, 1 falling) * max turns per second

// Current limit, in units of 10mA (current_limit of 100 --> 1A actual limit)
#define current_limit 1100 // 1100 --> 11A. This is approx 1/7 of 77A stall current, ~= max efficiency current for motor


// Calibration variables
int null_zone = 50;          // Threshold to reduce jitter at neutral stick position. 
                             // A value of 50 represents a null band of +/- 10% of the total power. This is to prevent stationary motor burn out of cheap motors.

int l_vert_center = 516;    // Read with stick in neutral position
int l_vert_adj_min = -514;  // Minimum value **relative** to center
int l_vert_adj_max = 503;   // Maximum value **relative** to center

int l_horiz_center = 505;

int r_vert_center = 497;
int r_vert_adj_min = -495;
int r_vert_adj_max = 521;

int r_horiz_center = 504;


// Variables assigned during loop()
int l_vert, l_horiz, r_vert, r_horiz;
long int qpps_set_left, qpps_set_right;


/* format_input takes an analogRead value and the calibration
 *  information corresonding to an axis for one of the joysticks.
 *  analog_in should be [0..1024] and the output will be 
 *  +/-0-100 %, with negative representing backwards.
 */
int format_input(int analog_in, int center_val, int min_val, int max_val) {
  long difference = analog_in - center_val;
  if (abs(difference) < null_zone) { return 0; }                           // Null zone prevents jitter in neutral position
  else if (difference < 0)         { return (-100)*difference/min_val; }   // Return negative 0-100% for backward movement
  else                             { return 100*difference/max_val; }      // Return positive 0-100% for forward movement   
}

//Setup communcations with roboclaw. Use pins 10 and 11 with 10ms timeout. Pin 10 goes to S2 on roboclaw; Pin 11 to S1
RoboClaw roboclaw(10,11,10000);


void setup() {
  // Open roboclaw serial communication
  roboclaw.begin(9600);
  Serial.begin(9600);
  
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
  

  // Set pins to read from joysticks
  pinMode(A0, INPUT);     // Left vertical
//  pinMode(A1, INPUT);   // Left horizontal
  pinMode(A2, INPUT);     // Right vertical
//  pinMode(A3, INPUT);   // Right horizontal
}

void loop() {
  l_vert = format_input(analogRead(A0), l_vert_center, l_vert_adj_min, l_vert_adj_max);   // Read l_vert axis and format analog read value into percentage
  r_vert = format_input(analogRead(A2), r_vert_center, r_vert_adj_min, r_vert_adj_max);   // Read r_vert axis and format analog read value into percentage

  Serial.print("Left: ");
  Serial.println(l_vert);
  Serial.print("Right: ");
  Serial.println(r_vert);
  

  qpps_set_left = qpps_max*l_vert/100; // Set percentage of max qpps
  qpps_set_right = qpps_max*r_vert/100; // Set percentage of max qpps
  
  roboclaw.SpeedM1(left_motor,-qpps_set_left); // Output to motor
  roboclaw.SpeedM2(left_motor,-qpps_set_left); // Output to motor

  roboclaw.SpeedM1(right_motor,qpps_set_right); // Output to motor
  roboclaw.SpeedM2(right_motor,qpps_set_right); // Output to motor  
}
