#include <ros/ros.h>
#include "leg_kinematics.hpp"

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;

double loop_freq = 100 ;
ros::Rate loop_rate(loop_freq); //HZ
#pragma endregion

while(ros::ok()){
  ros::spinOnce(); // Process any callbacks
  loop_rate.sleep();
}
return 0;
}