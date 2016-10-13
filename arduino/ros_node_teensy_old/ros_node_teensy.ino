/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#define vision_serial Serial1
#define drivetrain_serial Serial2
#define excavation_serial Serial3

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <sensor_msgs/Joy.h>

#define TIMEOUT 5000
#define RESETPIN 11
#define VOLTAGEPIN 3
unsigned long start_time;
char timeout_msg[35] = "Timeout while waiting for response";
bool powerState;

ros::NodeHandle nh;

std_msgs::String error_msg;
ros::Publisher error_msg_pub("arduino_error", &error_msg);

std_msgs::Int16 diagnostic_msg;
ros::Publisher diagnostic_msg_pub("arduino_diagnostic", &diagnostic_msg);

byte left_val, right_val, vision_high, vision_low, exc_auto, exc_man;

void joy_msg_handler( const sensor_msgs::Joy& joy_msg ){

  // Drive train code
  // Scale [-1, 1] float by 64 and convert to int
  left_val = (byte) int(joy_msg.axes[1] * 63 + 64);
  left_val = left_val | 0x80;

  diagnostic_msg.data = left_val;
  diagnostic_msg_pub.publish(&diagnostic_msg);
  
  right_val = (byte) int(joy_msg.axes[4] * 63 + 64);
  right_val = right_val & ~0x80;


  // Excavation code
  exc_auto = exc_auto | 0x80;
  
  if (joy_msg.buttons[0] == joy_msg.buttons[1]){ // Excavator up/down
    exc_man = exc_man & ~0x03;
  } else if (joy_msg.buttons[1] == 1) {
    exc_man = exc_man | 0x03;
  } else {
    exc_man = (exc_man | 0x02) & ~0x01;
  }

  // Bucket Up/Down (Up Down arrow)
  if (joy_msg.axes[7] == -1.0) {
    exc_man = exc_man | 0x0C;
  } else if (joy_msg.axes[7] == 1.0) {
    exc_man = (exc_man | 0x08) & ~0x04;
  } else {
    exc_man = exc_man & ~0x0C;
  }
  
  // Bucket Ladder and  Conveyor (LT + LB + RB)
  if ((joy_msg.axes[2] < 0.0) == joy_msg.buttons[4]){ // Bucket Ladder move
    exc_man = exc_man & ~0x70;
  } else if (joy_msg.buttons[4] == 1) {
    exc_man = exc_man | 0x70;
  } else {
    exc_man = (exc_man | 0x60) & ~0x10;
  }


  // Resume all autonomous routines
  if (joy_msg.buttons[7] == 1) {
    exc_auto = exc_auto & ~0x40;
  }

  // Stop all autonomous routines (flag, use start to clear)
  if (joy_msg.buttons[6] == 1) {
    exc_auto = exc_auto | 0x40;
  }

  writeDrivetrain();
  writeExcavation();
}

void excavation_state_msg_handler( const std_msgs::Byte& excavation_state ){
  byte x = excavation_state.data;
  
  exc_auto = (exc_auto & 0x60) | (x & 0xCF); // Updates desired state of arduino
  
  writeExcavation(); 
}


void kinect_servo_msg_handler( const std_msgs::Int16& kinect_servo_msg ){
  vision_serial.write(1);
  vision_serial.write(kinect_servo_msg.data);

  start_time = millis();
  while (!vision_serial.available()) { 
    if (millis() - start_time > TIMEOUT) {
      error_msg.data = timeout_msg;
      return;      error_msg_pub.publish(&error_msg);

    }
  }
  
  int return_value = vision_serial.read();
  if (return_value != 0) {
    if (return_value == 1) {
      char error[33] = "Kinect servo angle must be <= 90";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    } else { // return_value == 2
      char error[34] = "Kinect servo angle must be >= -90";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    }
    
  } 
}

void camera_servo_pan_msg_handler( const std_msgs::Int16& camera_servo_pan_msg ){
  vision_serial.write(2);
  vision_serial.write(camera_servo_pan_msg.data);

  start_time = millis();
  while (!vision_serial.available()) { 
    if (millis() - start_time > TIMEOUT) {
      error_msg.data = timeout_msg;
      error_msg_pub.publish(&error_msg);
      return;
    }
  }
  
  int return_value = vision_serial.read();
  if (return_value != 0) {
    if (return_value == 1) {
      char error[38] = "Camera pan servo angle must be <= 180";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    } else { // teturnValue == 2
      char error[39] = "Camera pan servo angle must be >= -180";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    }
    
  } 
}

void camera_servo_tilt_msg_handler( const std_msgs::Int16& camera_servo_tilt_msg ){
  vision_serial.write(3);
  vision_serial.write(camera_servo_tilt_msg.data);

  start_time = millis();
  while (!vision_serial.available()) { 
    if (millis() - start_time > TIMEOUT) {
      error_msg.data = timeout_msg;
      error_msg_pub.publish(&error_msg);
      return;
    }
  }
  
  int return_value = vision_serial.read();
  if (return_value != 0) {
    if (return_value == 1) {
      char error[38] = "Camera tilt servo angle must be <= 90";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    } else { // teturnValue == 2
      char error[39] = "Camera tilt servo angle must be >= -90";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    }
  } 
}

void writeDrivetrain() {
  drivetrain_serial.write(left_val);
  drivetrain_serial.write(right_val);
}

void writeExcavation() {
  excavation_serial.write(exc_auto);
  excavation_serial.write(exc_man);
}

void writeVision() {
  vision_serial.write(vision_high);
  vision_serial.write(vision_low);
}
ros::Subscriber<sensor_msgs::Joy> joy_sub("joyslow", &joy_msg_handler);
ros::Subscriber<std_msgs::Byte> excavation_state_sub("excavation_state", &excavation_state_msg_handler);
ros::Subscriber<std_msgs::Int16> kinect_servo_sub("kinect_servo", &kinect_servo_msg_handler);
ros::Subscriber<std_msgs::Int16> camera_servo_pan_sub("camera_servo_pan", &camera_servo_pan_msg_handler);
ros::Subscriber<std_msgs::Int16> camera_servo_tilt_sub("camera_servo_tilt", &camera_servo_tilt_msg_handler);


void setup()
{
  //Serial.begin(115200);
  vision_serial.begin(115200);
  drivetrain_serial.begin(115200);
  excavation_serial.begin(115200);
  nh.initNode();
  nh.advertise(error_msg_pub);
  nh.advertise(diagnostic_msg_pub);
  nh.subscribe(kinect_servo_sub);
  nh.subscribe(camera_servo_pan_sub);
  nh.subscribe(camera_servo_tilt_sub);
  nh.subscribe(joy_sub);
  nh.subscribe(excavation_state_sub);
  pinMode(VOLTAGEPIN, INPUT_PULLUP);
  pinMode(RESETPIN, OUTPUT);
  powerState = digitalRead(VOLTAGEPIN);
  digitalWrite(RESETPIN, HIGH);
}

void loop()
{
  if (powerState != digitalRead(VOLTAGEPIN)) {
    if (powerState == true) { // Falling edge, power lost to other arduinos
      char error[11] = "Power lost";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    } else { // Rising edge, power restored
      digitalWrite(RESETPIN, LOW);
      delayMicroseconds(5);
      digitalWrite(RESETPIN, HIGH);
      char error[15] = "Power regained";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    }
    powerState = digitalRead(VOLTAGEPIN);
  }
  nh.spinOnce();
}
