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
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#define TIMEOUT 5000
#define RESETPIN 11
#define VOLTAGEPIN 3
#define LED 13
unsigned long start_time;
char timeout_msg[35] = "Timeout while waiting for response";
bool powerState;

ros::NodeHandle nh;

std_msgs::String error_msg;
ros::Publisher error_msg_pub("arduino_error", &error_msg);

std_msgs::Int16 diagnostic_msg;
ros::Publisher diagnostic_msg_pub("arduino_diagnostic", &diagnostic_msg);

// Wheel current publisher
std_msgs::String whcurrent_msg;
ros::Publisher whcurrent_msg_pub("wh_current", &whcurrent_msg);

// Excavation current publisher
std_msgs::String excurrent_msg;
ros::Publisher excurrent_msg_pub("ex_current", &excurrent_msg);

// Vision Position Publishers
std_msgs::Byte camera_servo_pan_state;
ros::Publisher camera_servo_pan_state_pub("camera_servo_pan_state", &camera_servo_pan_state);

std_msgs::Byte camera_servo_tilt_state;
ros::Publisher camera_servo_tilt_state_pub("camera_servo_tilt_state", &camera_servo_tilt_state);


byte left_val, right_val, vision_high, vision_low, exc_auto, exc_man;
float lmult, rmult;

void joy_msg_handler( const sensor_msgs::Joy& joy_msg ){

  // Drive train code
  // Left Stick Controls
  // Reduces speed by half if stick button is pressed
  if(joy_msg.buttons[9] == 1) //L3 is 9
  	lmult = 0.5;
  else
	  lmult = 1;
    
  if(joy_msg.buttons[10] == 1) // R3 is 10
        rmult = 0.5;
  else
        rmult = 1;
        
  // Scales message from 1-127 (0 is not possible, 64 is stop) 
  left_val = (byte) int(lmult * joy_msg.axes[1] * 63.0 + 64.0);
  right_val = (byte) int(rmult * joy_msg.axes[4] * 63.0 + 64.0);

  // Label left wheel message with 1 in most significant bit 
  left_val = left_val | 0x80;
  
  // Label right wheel message with 0 in most significant bit
  right_val = right_val & ~0x80;

  // Writes data to diagnostic message if  written
  //diagnostic_msg.data = left_val;
  //diagnostic_msg_pub.publish(&diagnostic_msg); 


  // Excavation code

  // Excavator up/down (A-0 + B-1 + X-2)
  if (joy_msg.buttons[0] + joy_msg.buttons[1] + joy_msg.buttons[2] != 1){ 
    exc_man = exc_man & ~0x43;
  } else if (joy_msg.buttons[1] == 1) {
    exc_man = (exc_man | 0x03) & (~0x40); // Up
  } else if (joy_msg.buttons[2] == 1) {
    exc_man = ((exc_man | 0x42) & (~0x01)); // Slow Down
  } else {
    exc_man = (exc_man | 0x02) & (~0x41); // Quick Down
  }

  // Bucket Up/Down (RT-a5 + RB-5)
  if ((joy_msg.axes[5] < 0.0) == (joy_msg.buttons[5] == 1)) {
    exc_man = exc_man & ~0x0C; //Nothing
  } else if (joy_msg.buttons[5] == 1) {
    exc_man = (exc_man | 0x08) & ~0x04; // Up
  } else {
    exc_man = exc_man | 0x0C; // Down
  }
  
  // Bucket Ladder and  Conveyor (LT-a2 + LB-4)
  if ((joy_msg.axes[2] < 0.0) == joy_msg.buttons[4]){ // Bucket Ladder move
    exc_man = exc_man & ~0x30;
  } else if (joy_msg.buttons[4] == 1) {
    exc_man = exc_man | 0x30; // Reverse
  } else {
    exc_man = (exc_man | 0x20) & ~0x10; // Forward
  }


  // Resume all autonomous routines (Start-7)
  if (joy_msg.buttons[7] == 1) {
    exc_auto = exc_auto & ~0x40; 
  }

  // Stop all autonomous routines (flag, use start to clear) (Back-6)
  if (joy_msg.buttons[6] == 1) {
    exc_auto = exc_auto | 0x40;
  }

  // Set most significant bit to 1 on autonomy commands
  // Most significant bit on manual controls must be zero
  exc_auto = exc_auto | 0x80; 
  exc_man = exc_man & ~0x80;

  writeDrivetrain();
  writeExcavation();
}

void excavation_state_msg_handler( const std_msgs::Byte& excavation_state ){
  byte x = excavation_state.data;

  // Sends the state that the program would like the arduino in.

  // Bit 7 is 1 for byte identification, Bit 6 is enable or disable, bits 5-0 are the state wanted.
  exc_auto = (exc_auto & 0xC0) | (x & 0xCF); // Updates desired state of arduino
  
  writeExcavation(); 
}

void kinect_servo_msg_handler ( const std_msgs::Byte& kinect_servo_msg ){
  // Sets bits to control pan angle of kinect
  if(kinect_servo_msg.data < 4) {
    byte shift = kinect_servo_msg.data << 5; // Shift 5 (bits 6-5)
    vision_high = (vision_high & ~ 0x60) | (shift & 0x60);
    
  } else {
    char error[20];
    sprintf(error, "Invalid Kinect State: %i", kinect_servo_msg.data);
    error_msg.data = error;
    error_msg_pub.publish(&error_msg);
    
  }

  // Set highest bit of high byte to 1
  vision_high = vision_high | 0x80;
  
  writeVision();
  
}

void camera_servo_pan_msg_handler( const std_msgs::Byte& camera_servo_pan_msg ){
  // Sets bits to control pan angle of camera
  if(camera_servo_pan_msg.data < 64) {
    byte shift = camera_servo_pan_msg.data << 1; // Shift 1 (bits 6-1)
    vision_low = (vision_low & ~ 0x81) | (shift & 0x81); 
  } else {
    char error[20];
    sprintf(error, "Invalid Pan State: %i", camera_servo_pan_msg.data);
    error_msg.data = error;
    error_msg_pub.publish(&error_msg); 
  }

  // Set highest bit of low byte to 0
  vision_low = vision_low & 0x7F;
    
  writeVision();
  
}

void camera_servo_tilt_msg_handler( const std_msgs::Byte& camera_servo_tilt_msg ){
  // Sets bits to control tilt angle of camera
  if(camera_servo_tilt_msg.data < 32) {
    byte shift = camera_servo_tilt_msg.data; // Shift 0 (bits 4-0)
    vision_high = (vision_high & ~ 0x1F) | (shift & 0x1F); 
  } else {
    char error[20];
    sprintf(error, "Invalid Tilt State: %i", camera_servo_tilt_msg.data);
    error_msg.data = error;
    error_msg_pub.publish(&error_msg); 
  }

  // Set highest bit of high byte to 1
  vision_high = vision_high | 0x80;

  writeVision();
}

void camera_servo_raise_msg_handler( const std_msgs::Bool& camera_servo_raise_msg ){

  // Sets bit to control whether camera is raising or stops (no reverse) (bit 0)
  bool msg = camera_servo_raise_msg.data;
  if (msg) {
    vision_low = vision_low | 0x01;
  } else {
    vision_low = vision_low & ~ 0x01;
  }
  
  // Set highest bitof low byte to 0
  vision_low = vision_low & ~0x80;

  writeVision();
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

  // Byte Codes for controlling camera system (currently interpreted at arduino level, but not wired in)
  // Key: K = Kinect, T = Camera tilt, P = Camera Pan, R = Camera Raise
  // Vision high byte
  // b1KKTTTTT
  // Vision low byte
  // b0PPPPPPR
  
  vision_serial.write(vision_high);
  vision_serial.write(vision_low);
}


// Read current over serial and publish over ROS
void read_whcurrent(){
  //Read the incoming string and publish
  digitalWrite(LED, HIGH);
  char wh_curr_msg[30];
  drivetrain_serial.readBytesUntil('\n', wh_curr_msg, 30); //Read the incoming current data
  digitalWrite(LED, LOW);
  whcurrent_msg.data = wh_curr_msg; 
  whcurrent_msg_pub.publish(&whcurrent_msg);
      
}

// Read excavation current over serial and publish over ROS
void read_excurrent(){
  //Read the incoming string and publish
  digitalWrite(LED, HIGH);
  char ex_curr_msg[30];
  excavation_serial.readBytesUntil('\n', ex_curr_msg, 30); //Read the incoming current data
  digitalWrite(LED, LOW);
  excurrent_msg.data = ex_curr_msg; 
  excurrent_msg_pub.publish(&excurrent_msg);
}

void read_cam_data(){
  
  //Read the incoming string and publish
  digitalWrite(LED, HIGH);
  char cam_msg[30];
  // Looks for end marker (@) in string (ensure all written data has end marker in arduino code)
  int len = vision_serial.readBytesUntil('@', cam_msg, 30);
  digitalWrite(LED, LOW);
  
  if (len == 2) {
    camera_servo_pan_state.data = cam_msg[0];
    camera_servo_tilt_state.data = cam_msg[1];
    camera_servo_pan_state_pub.publish(&camera_servo_pan_state);
    camera_servo_tilt_state_pub.publish(&camera_servo_tilt_state);
  } else { 
    error_msg.data = cam_msg;
    error_msg_pub.publish(&error_msg);        
  }
  
  
}

// All ROS subscribers
ros::Subscriber<sensor_msgs::Joy> joy_sub("joyslow", &joy_msg_handler);
ros::Subscriber<std_msgs::Byte> excavation_state_sub("excavation_state", &excavation_state_msg_handler);
ros::Subscriber<std_msgs::Byte> kinect_servo_sub("kinect_servo", &kinect_servo_msg_handler);
ros::Subscriber<std_msgs::Byte> camera_servo_pan_sub("camera_servo_pan", &camera_servo_pan_msg_handler);
ros::Subscriber<std_msgs::Byte> camera_servo_tilt_sub("camera_servo_tilt", &camera_servo_tilt_msg_handler);
ros::Subscriber<std_msgs::Bool> camera_servo_raise_sub("camera_servo_raise", &camera_servo_raise_msg_handler);


void setup()
{
  //Serial.begin(115200);
  vision_serial.begin(115200);
  drivetrain_serial.begin(115200);
  excavation_serial.begin(115200);

  // Start ROS
  nh.initNode();
  
  // Advertise publishers
  nh.advertise(error_msg_pub);
  nh.advertise(diagnostic_msg_pub);
  nh.advertise(whcurrent_msg_pub);
  nh.advertise(excurrent_msg_pub);
  nh.advertise(camera_servo_pan_state_pub);
  nh.advertise(camera_servo_tilt_state_pub);

  // Subscribe Subscribers
  nh.subscribe(kinect_servo_sub);
  nh.subscribe(camera_servo_pan_sub);
  nh.subscribe(camera_servo_tilt_sub);
  nh.subscribe(camera_servo_raise_sub);
  nh.subscribe(joy_sub);
  nh.subscribe(excavation_state_sub);
  //pinMode(VOLTAGEPIN, INPUT_PULLUP);
  //pinMode(RESETPIN, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  //powerState = digitalRead(VOLTAGEPIN);
  //digitalWrite(RESETPIN, HIGH);
  
}

void loop()
{
  if (powerState != digitalRead(VOLTAGEPIN)) {
    if (powerState == true) { // Falling edge, power lost to other arduinos
      char error[11] = "Power lost";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    } else { // Rising edge, power restored
      /*digitalWrite(RESETPIN, LOW);
      delayMicroseconds(5);
      digitalWrite(RESETPIN, HIGH);*/
      char error[15] = "Power regained";
      error_msg.data = error;
      error_msg_pub.publish(&error_msg);
    }
    powerState = digitalRead(VOLTAGEPIN);
  }
  
  // Read and publish the current from the wheels
  if( drivetrain_serial.available() > 0){
    read_whcurrent();
  }
  
  // Read and publish data from the excavation system
  if(excavation_serial.available() > 0){
    read_excurrent();   
  }

  // Read and publish camera state from the vision system
  if( vision_serial.available() > 0){
    read_cam_data();   
  }
  
  nh.spinOnce();
}
