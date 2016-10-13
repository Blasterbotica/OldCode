#include <new>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <cstdlib>
#include <stdio.h>

int main(int argc, char** argv){
  //ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
    //ROS_INFO("Waiting for the move_base action server to come up");



int move_array[5][3] = {
  {5,5,1}, //First waypoint
  {6,9,1}, //Second waypoint
  {6,9,1}, //Third
  {6,9,1}, //Fourth
  {1,1,1}, //Return to base
  };

int x[5]= {5,6,7,8,9};

for (int i=0; i<4; i++){
int x = move_array[i][0];
int y = move_array[i][1];
int mode = move_array[i][2];
 printf("x: %d y: %d mode: %d\n",move_array[i][0],move_array[i][1],move_array[i][2]);
 printf("x: %d y: %d mode: %d\n",x,y,mode);
  //printf("y vaue %d\n",move_array[i][1]);
}

//printf("Hi");
int y = 5;

printf("hello world %d\n",move_array[0][1]);
std::cout << "hello world\n";
//std::cout << x[5];

}
