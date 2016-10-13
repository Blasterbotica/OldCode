/* This code is intended for the Arduino Uno built into the
 *  prototype controller for the drivetrain platform. It 
 *  uses analog input read from the vertical axis of each 
 *  joystick and converts it to a value that is written to 
 *  a geometry_msgs/Twist data type used to control the
 *  Turtlebot. The node is set up to publish to the
 *  /cmd_vel_mux/input/teleop topic, to which the Turtlebot
 *  is subscribed.
 */

#include<ros.h>
#include<geometry_msgs/Twist.h>

// ROS stuff
ros::NodeHandle nh; // Instantiate node handler

geometry_msgs::Twist cmd_msg; // Instantiate message variable of type geometry_msgs/Twist
ros::Publisher msg_publisher("/cmd_vel_mux/input/teleop", &cmd_msg); // Instantiate publisher that publishes to /cmd_vel_mux/input/teleop topic with messages of type dictated by cmd_msg
 
// Calibration variables
int null_zone = 2;          // Threshold to reduce jitter at neutral stick position

int l_vert_center = 516;    // Read with stick in neutral position
int l_vert_adj_min = -514;  // Minimum value relative to center
int l_vert_adj_max = 503;   // Maximum value relative to center

int r_vert_center = 497;    // Read with stick in neutral position
int r_vert_adj_min = -495;  // Minimum value relative to center
int r_vert_adj_max = 521;   // Maximum value relative to center


// Variables assigned during loop()
int l_vert, r_vert;

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

/*write_global_cmd_msg takes the left and right percentage values
 * read from the joysticks and converts them into a geometry_msgs/Twist
 * format variable used for controlling the Turtlebot. The values (+/-0.2, +/-1)
 * were taken from a keyboard teleop example used to control the Turtlebot
 * and were assumed to represent reasonable operating velocities for the
 * Turtlebot. At present, there is not analog control, meaning that the 
 * Turtlebot is either going full speed based on the stick positions or 
 * stopped. For future use, an acceleration governor and a higher linear speed
 * will make it significantly more pleasant to use. cmd_msg is a global variable
 * instantiated above.
 */
void write_global_cmd_msg(int left, int right) {
  if(left == 0) {
    if (right == 0)       { cmd_msg.linear.x = 0;     cmd_msg.angular.z = 0; }
    else if (right < 0)   { cmd_msg.linear.x = -0.2;  cmd_msg.angular.z = -1; }
    else                  { cmd_msg.linear.x = 0.2;   cmd_msg.angular.z = 1; }
  } else if (left < 0) {
    if (right == 0)       { cmd_msg.linear.x = -0.2;  cmd_msg.angular.z = 1; }
    else if (right < 0)   { cmd_msg.linear.x = -0.2;  cmd_msg.angular.z = 0; }
    else                  { cmd_msg.linear.x = 0;     cmd_msg.angular.z = 1; }
  } else {
    if (right == 0)       { cmd_msg.linear.x = 0.2;   cmd_msg.angular.z = -1; }
    else if (right < 0)   { cmd_msg.linear.x = 0;     cmd_msg.angular.z = -1; }
    else                  { cmd_msg.linear.x = 0.2;   cmd_msg.angular.z = 0; }
  }
}

void setup() {
  // Initialize the ROS node and define what type of message/topic name it will publish to
  nh.initNode();
  nh.advertise(msg_publisher);

  // Initialize input pins to read from controller
  pinMode(A0, INPUT);     // Left vertical
  pinMode(A2, INPUT);     // Right vertical
}

void loop() {
  
  l_vert = format_input(analogRead(A0), l_vert_center, l_vert_adj_min, l_vert_adj_max);   // Read l_vert axis and format analog read value into percentage
  r_vert = format_input(analogRead(A2), r_vert_center, r_vert_adj_min, r_vert_adj_max);   // Read r_vert axis and format analog read value into percentage
  
  write_global_cmd_msg(l_vert, r_vert); // Use the l and r vert percentages to write to the global variable, cmd_msg

  msg_publisher.publish(&cmd_msg); // Publish the global cmd_msg variable to the 
  nh.spinOnce(); // Required for proper function of ROS node
  
}
