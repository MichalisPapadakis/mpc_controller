#include <ros/ros.h>
#include "leg_kinematics.hpp"
#include <eigen3/Eigen/Dense>

//message_types
#include "cubemars_controller_ros_msgs/SetpointArray.h"
#include "sensor_msgs/JointState.h"


class leg_controller {
  public:		
    //TODO: initialize kinematis config, id
    leg_controller():
    kinematics(1),
    id(0),
    setpoint_arr(init_setpoint_array()),
    qd(setpoint_arr.values.data())
    {
      ros::NodeHandle nh;
      std::string leg_node_name = "leg1_node/";
      command_pub     = nh.advertise<cubemars_controller_ros_msgs::SetpointArray>(leg_node_name+"command_position",10);
      motor_state_sub = nh.subscribe("/joint_states",10,&leg_controller::status_callback,this);

      full_state.setZero();
      ros::spinOnce(); //Populate message with current state
      qd[0] = q[0];
      qd[1] = q[1];
      qd[2] = q[3];

    }

  void Publish(){
    setpoint_arr.header.stamp = ros::Time::now();
    command_pub.publish(setpoint_arr);
  }

  private:

  void status_callback(const sensor_msgs::JointStateConstPtr & joint_state ){
    
    for (int i=0;i<5;i++){
      q[i]  = joint_state->position[ joint_id[i] ];
      qt[i] = joint_state->velocity[ joint_id[i] ];
    }

  //Debuggin:
  // ROS_INFO_STREAM("Current state for FR: "<< full_state );

  }

  //This function is used to initialize `setpoint_arr` in the initializer list.
  //
  // --- 
  //
  // - It is needed so qd which is `Eigen::Map<Eigen::Vector3f>` can share the same memory with `setpoint_arr_.values`
  cubemars_controller_ros_msgs::SetpointArray init_setpoint_array(){
    //TODO: give correct motor ids based on leg
    // maybe use joint_id
    cubemars_controller_ros_msgs::SetpointArray setpoint_arr_;
    setpoint_arr_.ids.push_back(6);
    setpoint_arr_.ids.push_back(5);
    setpoint_arr_.ids.push_back(4);
    setpoint_arr_.values.assign(3,0);

    return setpoint_arr_; 

  }

  //TODO: give correct joint id from config
  std::array<int,5> joint_id{0,1,2,3,4}; 
  leg_kinematics kinematics;
  const int id; 
  cubemars_controller_ros_msgs::SetpointArray setpoint_arr;

  ros::Publisher command_pub;
  ros::Subscriber motor_state_sub;

  Eigen::Matrix<double,10,1> full_state;
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  q = full_state.head<5>() ;  //alias for position part
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  qt = full_state.tail<5>(); //alias for velocity part

  ///Desired joint position.
  ///@note Map takes a pointer to the data container to construct. Size specified by using Vector3f.
  ///This way efficiently uses eigen to interface with controller and the `setpoint_arr.position` vector to publish the msg
  Eigen::Map<Eigen::Vector3f> qd; 

};

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;

double loop_freq = 100 ;
ros::Rate loop_rate(loop_freq); //HZ
#pragma endregion


leg_controller controller ;

while(ros::ok()){
  controller.Publish();
  ros::spinOnce(); // Process any callbacks
  loop_rate.sleep();
}
return 0;
}