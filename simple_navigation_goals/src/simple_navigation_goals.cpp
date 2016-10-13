#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <ctime>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //Initialize the mode publisher node
  ros::init(argc, argv, "mode");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;


  //Set up velocity publisher
  //ros::Publisher _pub = _n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

  //Set up mode publisher node
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Build array of moves
  // x coordinate, y coordinate, mode
  int move_array [5][3] = {
  {5,5,1}, //First waypoint
  {4,3,1}, //Second waypoint
  {8,7,1}, //Third
  {6,9,1}, //Fourth
  {1,1,1}, //Return to base
  };


for (int i=0; i<5; i++){

 //Read off waypoint values and mode
    int x = move_array[i][0];
    int y = move_array[i][1];
    int mode = move_array[i][2];
    printf("x: %d y: %d mode: %d\n",x,y,mode);

 // Publish Mode
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Current mode is: " << mode;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
 // End Mode Publisher

  //Instruct robot to move to new position
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = move_array[i][0];
  goal.target_pose.pose.position.y = move_array[i][1];

  goal.target_pose.pose.orientation.w = i/3.14;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
 // End Waypoint Script

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Waypoint Acchieved");
  else
    ROS_INFO("The movement failed");
}

 //Back up to a bucket
	int tstart = time(0);
	while (-5<difftime(tstart,time(0))){
  	//geometry_msgs::Twist vel;
  	//vel.linear.x = -0.2;
	//vel.angular.z = 0.0;
	//_pub.publish(vel);
	printf("It's working! %f\n",difftime(tstart,time(0)));
	//sleep(200);
	}

ROS_INFO("Path Complete, waiting for next request");



  return 0;
}
