#include <ros/ros.h>
#include "leg_kinematics.hpp"
#include <eigen3/Eigen/Dense>

//message_types
#include "cubemars_controller_ros_msgs/SetpointArray.h"
#include "sensor_msgs/JointState.h"

#define corresponding_kinematic_configuration(leg_id) ( (i==0 || i==3)? 1: 2 ) 

//This are read from a config in WBC class
struct leg_controller_opt{
  std::array<int,5> joint_state_id{0,1,2,3,4};
  std::array<int,3> motor_id{6,5,4};
  int kinematic_config = 1; //[1,2]
  int leg_id = 0; 
};

class leg_controller {
  public:		
  leg_controller(const leg_controller_opt & opts):
  joint_state_id(opts.joint_state_id),
  kinematics(opts.kinematic_config),
  id(opts.leg_id),
  setpoint_arr(init_setpoint_array(opts.motor_id)),
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

      //Debugging:
      std::cout << "Current id is: " << id << std::endl;
      for (auto id:joint_state_id){std::cout << id << std::endl;}
      std::cout << "====" << std::endl;
      for (auto id:opts.motor_id){std::cout << id << std::endl;}

  }

  void Publish(){
    setpoint_arr.header.stamp = ros::Time::now();
    command_pub.publish(setpoint_arr);
  }

  private:

  void status_callback(const sensor_msgs::JointStateConstPtr & joint_state ){
    for (int i=0;i<5;i++){
      q[i]  = joint_state->position[ joint_state_id[i] ];
      qt[i] = joint_state->velocity[ joint_state_id[i] ];
    }

  //Debuggin:
  // ROS_INFO_STREAM("Current state for FR: "<< full_state );

  }

  //This function is used to initialize `setpoint_arr` in the initializer list.
  //
  // --- 
  //
  // - It is needed so qd which is `Eigen::Map<Eigen::Vector3f>` can share the same memory with `setpoint_arr_.values`
  cubemars_controller_ros_msgs::SetpointArray init_setpoint_array(const std::array<int,3> & motor_id) const {

    cubemars_controller_ros_msgs::SetpointArray setpoint_arr_;
    for (int id:motor_id){
      setpoint_arr_.ids.push_back(id);
      setpoint_arr_.values.push_back(0);
    }

    return setpoint_arr_; 

  }

  const int id; 
  const std::array<int,5> joint_state_id; 
  leg_kinematics kinematics;

  ros::Publisher command_pub;
  ros::Subscriber motor_state_sub;
  cubemars_controller_ros_msgs::SetpointArray setpoint_arr;

  Eigen::Matrix<double,10,1> full_state;
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  q = full_state.head<5>() ;  //alias for position part
  Eigen::Block<Eigen::Matrix<double,10,1>,5,1>  qt = full_state.tail<5>(); //alias for velocity part

  ///Desired joint position.
  ///@note Map takes a pointer to the data container to construct. Size specified by using Vector3f.
  ///This way efficiently uses eigen to interface with controller and the `setpoint_arr.position` vector to publish the msg
  Eigen::Map<Eigen::Vector3f> qd; 

};

class whole_body_controller {
  public:		
    whole_body_controller() {
      ros::NodeHandle nh;

      //get parameters:
      std::string param_name;

      std::array<std::string,4> leg_prefix{ "fr","rr","fl","rl"};
      std::array< std::vector<int>,4 > joint_state_ids;
      std::array< std::vector<int>,4 > motor_ids;

      for (int i=0; i<4; i++){
        param_name = leg_prefix[i]+"_joint_state_ids";
        if ( !nh.getParam(param_name,joint_state_ids[i]) ){
           ROS_INFO_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }

        param_name = leg_prefix[i]+"_motor_ids";
        if ( !nh.getParam(param_name,motor_ids[i]) ){
           ROS_INFO_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }
      }

      //create corresponding structs
      for (int i =0; i<4; i++){
        leg_controller_opt leg_opts;
        leg_opts.leg_id = i;
        leg_opts.kinematic_config = corresponding_kinematic_configuration(i) ;

        std::copy(motor_ids[i].begin(),motor_ids[i].end(),leg_opts.motor_id.begin());
        std::copy(joint_state_ids[i].begin(),joint_state_ids[i].end(),leg_opts.joint_state_id.begin());

        leg_controllers[i] = std::make_unique<leg_controller>(leg_opts);
      }

      ;
    }
  private:
  std::array< std::unique_ptr< leg_controller>,4> leg_controllers;
};

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;

double loop_freq = 100 ;
ros::Rate loop_rate(loop_freq); //HZ
#pragma endregion

whole_body_controller wbc;

while(ros::ok()){
  // controller.Publish();
  ros::spinOnce(); // Process any callbacks
  loop_rate.sleep();
}
return 0;
}