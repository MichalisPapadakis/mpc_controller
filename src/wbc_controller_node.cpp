#include <ros/ros.h>
#include "leg_kinematics.hpp"
#include <eigen3/Eigen/Dense>

//message_types
#include "cubemars_controller_ros_msgs/SetpointArray.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
#include "mpc_controller/phase.h"
#include "mpc_controller/mpc_status.h"

#define corresponding_kinematic_configuration(leg_id) ( (i==0 || i==3)? 1: 2 ) 
#define traj_stages 4

#include <stdlib.h>
// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// mpc planner includes:
#include "c_generated_code_SRBD/acados_solver_SRBD.h"
#include "c_generated_code_leg/acados_solver_fr_leg_torque.h"

// WBC defines:
#include "wbc_state_machine_params.hpp"
#define TH_L 0.5
#define TH_B 0.1
#define PRINT_LEG_PLANNER_MPC_RESULTS false
#define PRINT_BODY_PLANNER_MPC_RESULTS false
#define DEBUG_TRAJECTORY_PUBLISHING true


//This are read from a config in WBC class
struct leg_controller_opt{
  std::array<int,5> joint_state_id{0,1,2,3,4};
  std::array<int,3> motor_id{6,5,4};
  int kinematic_config = 1; //[1,2]
  int leg_id = 0; 
};


enum base_joint {weld, revolute_x, revolute_y, revolute_z, floating };

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
      std::string leg_node_name = "leg"+ std::to_string(id+1)+"_node/";
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
      std::cout <<"Kinematic config is:" << opts.kinematic_config << std::endl;
      std::cout << "====" << std::endl;
      for (auto id:setpoint_arr.ids){std::cout << int(id) << std::endl;}

  }

  void setCommand(const Eigen::Vector3f & qd_){
      qd = qd_;
      Publish();
  }

  void setCommand(const std::vector<float> & qd_vec){
      if (qd_vec.size()!=3){
        ROS_ERROR_STREAM("[mpc_controller]: wrong vector size. Abort command");
      }else{
        setpoint_arr.values = qd_vec;
        Publish();
      }
  }

  Eigen::Matrix<double,5,1> leg_state_estimation(const Eigen::Vector3d & qd){
    return kinematics.state_estimation(qd);
  }

  Eigen::Matrix<double,10,1>  get_full_state() const {
    return full_state; 
  }

  const Eigen::Matrix<double,10,1> *  get_full_state_ptr() const {
    return &full_state; 
  }

  private:

  void Publish(){
    setpoint_arr.header.stamp = ros::Time::now();
    command_pub.publish(setpoint_arr);
  }

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
      ros::NodeHandle nh_param("~");

      //1. get parameters:
      #pragma region
      std::string param_name;

      //Leg parameters:
      std::array<std::string,4> leg_prefix{ "fr","rr","fl","rl"};
      std::array< std::vector<int>,4 > joint_state_ids;
      std::array< std::vector<int>,4 > motor_ids;

      std::array<int,4> leg_order{1,3,0,2}; //default leg order
      std::vector<int> leg_order_;

      for (int i=0; i<4; i++){
        param_name = leg_prefix[i]+"_joint_state_ids";
        if ( !nh_param.getParam(param_name,joint_state_ids[i]) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }

        param_name = leg_prefix[i]+"_motor_ids";
        if ( !nh_param.getParam(param_name,motor_ids[i]) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: " << param_name);
        }
      }
      
      if ( !nh_param.getParam("leg_order",leg_order_) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: leg_order ");
      }else { 
        if (leg_order_.size()!=4){
          ROS_ERROR_STREAM("[mpc controller]: Expected 4 ids for leg order. Using default order ");
        }else{
          for(int i=0; i< 4; i++){leg_order[i] = leg_order_[i]; }
        }
      }

      //Ros topics:
      std::string body_current_pose_topic = "/qualisys/olympus/odom"; 
      std::string body_desired_pose_topic= "olympus/desired_angle";
      if ( !nh_param.getParam("body_current_pose_topic",body_current_pose_topic) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: body_current_pose_topic ");
      }

      if ( !nh_param.getParam("body_desired_pose_topic",body_desired_pose_topic) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: body_desired_pose_topic ");
      }
      
      //Controller options:
      int controller_mode_param = 0;
      if ( !nh_param.getParam("controller_mode",controller_mode_param) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: controller_mode. Selecting floating controller mode ");
      }else{
        if (controller_mode_param > 4){
          ROS_ERROR_STREAM("[mpc controller]: Wrong controller mode. Allowed are: [0: weld, 1: revolute_x, 2: revolute_y, 3: revolute_z, 4: floating] ");
        }else{
          controller_config = controller_mode_param; 
          ROS_INFO_STREAM( "[mpc controller]: Controller configuration is: " << controller_config );
        }
      }

      if ( !nh_param.getParam("stabilization_mode",stabilization_mode) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: stabilization_mode.  ");
      }
      std::vector<double> default_torque;
      if ( !nh_param.getParam("manoeuvre_target_torque",default_torque) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: manoeuvre_target_torque.  ");
      }else{
        if (default_torque.size()==3){
          for (int i=0;i<3;i++){ default_torque_target[i] = default_torque[i]; }
        }else {ROS_ERROR_STREAM("[mpc controller]: Default torque target must contain 3 elements, 2 of which 0: [tx ty tz] "); }        
      }

      std::vector<double> contracting_parameters;
      if ( !nh_param.getParam("contracting_param",contracting_parameters) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: contracting_param.  ");
      }else{
        if (contracting_parameters.size()==4){
          for (int i=0;i<4;i++){ contracting_param[i] = contracting_parameters[i]; }
        }else {ROS_ERROR_STREAM("[mpc controller]: Expected 4 contracting_parameters. "); }        
      }

      if ( !nh_param.getParam("angular_offsets/roll",ang_offset_roll) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: angular_offsets/roll. Using 45deg  ");
      }
      if ( !nh_param.getParam("angular_offsets/pitch",ang_offset_pitch) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: angular_offsets/pitch. Using 0deg  ");
      }
      if ( !nh_param.getParam("angular_offsets/yaw",ang_offset_yaw) ){
           ROS_ERROR_STREAM("[mpc controller]: Couldn't read parameter: angular_offsets/yaw. Using 0deg  ");
      }

      #pragma endregion
      
      //2. create corresponding leg controllers
      for (int i =0; i<4; i++){
        leg_controller_opt leg_opts;
        leg_opts.leg_id = leg_order[i];
        leg_opts.kinematic_config = corresponding_kinematic_configuration(i) ;

        std::copy(motor_ids[i].begin(),motor_ids[i].end(),leg_opts.motor_id.begin());
        std::copy(joint_state_ids[i].begin(),joint_state_ids[i].end(),leg_opts.joint_state_id.begin());

        leg_controllers[i] = std::make_unique<leg_controller>(leg_opts);
      }

      //Set Controller subscribers:
      olympus_current_pose_sub = nh.subscribe(body_current_pose_topic,10,&whole_body_controller::body_current_pose_update,this);
      olympus_desired_pose_sub = nh.subscribe(body_desired_pose_topic,10,&whole_body_controller::body_desired_pose_update,this);

      //3.  Reseting init
      phase_publisher = nh.advertise<mpc_controller::phase>("controller_current_phase",10);
      leg_planner_status_pub  = nh.advertise<mpc_controller::mpc_status>("leg_mpc_status",10);
      body_planner_status_pub = nh.advertise<mpc_controller::mpc_status>("body_mpc_status",10);

      //TODO: remove them after logic for mode:
      W_interrupt.setZero();
      resetting::populate_diagonal(W_interrupt,*( resetting::yaw_W          [current_phase] ) );
      resetting::populate_vector  (q_interrupt,*( resetting::yaw_setpoints  [current_phase] ) );
      Th_intterupt = resetting::yaw_thresholds [current_phase ];

      double conv = 180./M_PI;
      ROS_INFO("[wbc controller]: Current setpoint is: [%.1f,%.1f,%.1f]",q_interrupt[0]*conv,q_interrupt[1]*conv,q_interrupt[3]*conv);
      ROS_INFO("[wbc controller]: Current phase is : %d",current_phase);
      ROS_INFO("[wbc controller]: Current threshold  is : %.1f",Th_intterupt);

      //4.  Controller Timers:
      body_planner_timer = nh.createTimer(ros::Duration(TH_B),&whole_body_controller::body_planner_update,this); 
      leg_planner_timer  = nh.createTimer(ros::Duration(TH_L),&whole_body_controller::leg_planner_update,this);
      trajectory_timer   = nh.createTimer(ros::Duration(TH_L/FR_LEG_TORQUE_N),&whole_body_controller::trajectory_publish,this);

      // trajectory_timer.stop();
      // leg_planner_timer.stop();
      //MPC initialization:
      body_planner_solver_init();
      leg_planner_solver_init();
    }

  private:

  // ===== SUBSCRIBERS CALLBACKS ========

  void body_current_pose_update(const nav_msgs::OdometryConstPtr & odom){
    const double & quat_w = odom->pose.pose.orientation.w;
    const double & quat_x = odom->pose.pose.orientation.x;
    const double & quat_y = odom->pose.pose.orientation.y;
    const double & quat_z = odom->pose.pose.orientation.z;

    const double & w_x = odom->twist.twist.angular.x;
    const double & w_y = odom->twist.twist.angular.y;
    const double & w_z = odom->twist.twist.angular.z;

    quat_current.w()  = odom->pose.pose.orientation.w;
    
    // Switch reading
    switch (controller_config){
      case base_joint::weld : 
        quat_current.setIdentity();
        break;
      case base_joint::revolute_x : // Roll  axis is the z axis of the mo-cap
        quat_current.vec() << quat_z,quat_x,quat_y;
        twist_current      <<    w_z,   w_x,   w_y;
        break;
      case base_joint::revolute_y : // Pitch axis is the z axis of the mo-cap
        quat_current.vec() << quat_y,quat_z,quat_x;
        twist_current      <<    w_y,   w_z,   w_x;
        break;
      default: //floating & revolute z: // Either from drake or yaw axis == z axis of mo-cap
        quat_current.vec() << quat_x,quat_y,quat_z;
        twist_current      <<    w_x,   w_y,   w_z;
    }

    

  }
  
  void body_desired_pose_update(const std_msgs::Float32MultiArrayConstPtr & des){
    if (des->data.size() == 1){
      // For experiments
      Eigen::Vector3d  axis( 0,0,1) ;
      std::string axis_name = "z";

      switch (controller_config){
        case base_joint::revolute_x : // Roll  axis is the z axis of the mo-cap
          axis << 1,0,0;
          axis_name = "x";
          break;
        case base_joint::revolute_y : // Pitch axis is the z axis of the mo-cap
          axis << 0,1,0;
          axis_name = "y";
          break;
        default: //weld floating & revolute z: 
          break;
      }
      ROS_INFO_STREAM("[wbc controller]: One value passed as desired orientation. Assuming angle in "<< axis_name <<" axis");
      ROS_INFO_STREAM("[wbc controller]: Desired angle is "<< des-> data[0]);
      
      double angle = des-> data[0] * M_PI/180;
      Eigen::AngleAxisd qinput( angle, axis);
      quat_desired = Eigen::Quaterniond(  qinput);
    }
    else if (des->data.size() == 4){
      //Mainly for drake
      ROS_INFO_STREAM("[wbc controller]: 4 values passed as desired orientation. Assuming angle-axis notation.");
      Eigen::Vector3d  axis( des->data[0], des->data[1], des->data[2]) ; 
      double angle = des-> data[3] * M_PI/180;
      Eigen::AngleAxisd qinput( angle, axis);
      quat_desired = Eigen::Quaterniond(  qinput);
    }else{
      ROS_ERROR_STREAM("[wbc controller]: Expected 4 values (axis,angle) for desired target");

    }
  }


  // ===== BODY PLANNER METHODS ========
  private:
  void body_planner_solver_init(){
    acados_ocp_capsule_SRBD = SRBD_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    N_b = SRBD_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = SRBD_acados_create_with_discretization(acados_ocp_capsule_SRBD, N_b, new_time_steps);

    if (status)
    {
        printf("SRBD_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    nlp_config_SRBD = SRBD_acados_get_nlp_config(acados_ocp_capsule_SRBD);
    nlp_dims_SRBD = SRBD_acados_get_nlp_dims(acados_ocp_capsule_SRBD);
    nlp_in_SRBD = SRBD_acados_get_nlp_in(acados_ocp_capsule_SRBD);
    nlp_out_SRBD = SRBD_acados_get_nlp_out(acados_ocp_capsule_SRBD);
    nlp_solver_SRBD = SRBD_acados_get_nlp_solver(acados_ocp_capsule_SRBD);
    nlp_opts_SRBD = SRBD_acados_get_nlp_opts(acados_ocp_capsule_SRBD);

    for (int id=0; id< SRBD_NBX0 ; id++){ idxbx_b[id]=id; }

    //init corresponding variables:
    quat_current.setIdentity();
    quat_desired.setIdentity();
    twist_current.setZero();

    xtraj_b.fill(0);
    utraj_b.fill(0);
  }

  void body_planner_update_constraints(double *x0){
    ocp_nlp_constraints_model_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_in_SRBD, 0, "idxbx", idxbx_b.data());
    ocp_nlp_constraints_model_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_in_SRBD, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_in_SRBD, 0, "ubx", x0);

  }

  void body_planner_initialize_solution(Eigen::Matrix<double,SRBD_NX,1> & x0){
    Eigen::Vector3d u0;
    u0.setZero();

    for (int i = 0; i < N_b; i++)
    {
        ocp_nlp_out_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_out_SRBD, i, "x", x0.data());
        ocp_nlp_out_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_out_SRBD, i, "u", u0.data());
    }
    ocp_nlp_out_set(nlp_config_SRBD, nlp_dims_SRBD, nlp_out_SRBD, N_b, "x", x0.data());
  }

  public:
  void body_planner_update(const ros::TimerEvent&){
    //Convert to [w,v] quaternion convetion:
    Eigen::Matrix<double,4,1> q_desired_conv; //[w,v] convention
    q_desired_conv[0] = quat_desired.w();
    q_desired_conv.tail(3) = quat_desired.vec();

    Eigen::Matrix<double,SRBD_NX,1> body_state; //[w,v,angular]
    body_state[0] = quat_current.w();
    body_state.segment(1,3) = quat_current.vec();
    body_state.tail(3)      = twist_current;


    // set parameters - reference quaternions
    for (int ii = 0; ii <= N_b; ii++)
    {
        SRBD_acados_update_params(acados_ocp_capsule_SRBD, ii, q_desired_conv.data(), SRBD_NP);
    }

    body_planner_update_constraints(body_state.data());

    body_planner_initialize_solution(body_state);

    //update config
    ocp_nlp_solver_opts_set(nlp_config_SRBD, nlp_opts_SRBD, "rti_phase", &body_planner_counter);

    //Solve: 
    int status = SRBD_acados_solve(acados_ocp_capsule_SRBD);

    double elapsed_time;
    //Get data:
    ocp_nlp_get(nlp_config_SRBD, nlp_solver_SRBD, "time_tot", &elapsed_time);
    elapsed_time = MIN(elapsed_time, 1e12);

    //get solution
    for (int ii = 0; ii <= nlp_dims_SRBD->N; ii++)
        ocp_nlp_out_get(nlp_config_SRBD, nlp_dims_SRBD, nlp_out_SRBD, ii, "x", &xtraj_b[ii*SRBD_NX]);
    for (int ii = 0; ii < nlp_dims_SRBD->N; ii++)
        ocp_nlp_out_get(nlp_config_SRBD, nlp_dims_SRBD, nlp_out_SRBD, ii, "u", &utraj_b[ii*SRBD_NU]);

    body_planner_mpc_status.header.stamp = ros::Time::now();
    body_planner_mpc_status.status = status;
    body_planner_mpc_status.solution_time = elapsed_time;

    //TODO: remove this
    if (PRINT_BODY_PLANNER_MPC_RESULTS){
      printf("\n--- xtraj ---\n");
      d_print_exp_tran_mat( SRBD_NX, N_b+1, xtraj_b.data(), SRBD_NX);
      printf("\n--- utraj ---\n");
      d_print_exp_tran_mat( SRBD_NU, N_b, utraj_b.data(), SRBD_NU );
    }

    if (stabilization_mode){
      double CV = -1; //for yaw

      tref_mh << CV*utraj_b[0],CV*utraj_b[1], SATURATE_YAW(CV*utraj_b[2]) ;
      if (current_mode == controller_mode::yaw ){
        if ( quat_desired.angularDistance(quat_current)*180./M_PI > 25) { 

          is_direction_positive = ( tref_mh[2] > 0 ) ? true : false;
          
          if ( is_direction_positive){ tref_mh[2] =  YAW_MAX_POS_TRACKING_TORQUE; }
          if (!is_direction_positive){ tref_mh[2] =  YAW_MAX_NEG_TRACKING_TORQUE; }
        }
      }
      
      
      ROS_INFO("tref_body = [%.1f,%.1f,%.1f]",utraj_b[0],utraj_b[1],utraj_b[2]);
      ROS_INFO("tref_mpc = [%.1f,%.1f,%.1f]",tref_mh[0],tref_mh[1],tref_mh[2]);
    }



  }

  // ===== LEG PLANNER METHODS ========
  private:
  /// @brief This function creates the solver data types 
  void leg_planner_solver_init(){
    acados_ocp_capsule_leg = fr_leg_torque_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    N_l = FR_LEG_TORQUE_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = fr_leg_torque_acados_create_with_discretization(acados_ocp_capsule_leg, N_l, new_time_steps);

    if (status)
    {
        printf("fr_leg_torque_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    nlp_config_leg = fr_leg_torque_acados_get_nlp_config(acados_ocp_capsule_leg);
    nlp_dims_leg = fr_leg_torque_acados_get_nlp_dims(acados_ocp_capsule_leg);
    nlp_in_leg = fr_leg_torque_acados_get_nlp_in(acados_ocp_capsule_leg);
    nlp_out_leg = fr_leg_torque_acados_get_nlp_out(acados_ocp_capsule_leg);
    nlp_solver_leg = fr_leg_torque_acados_get_nlp_solver(acados_ocp_capsule_leg);
    nlp_opts_leg = fr_leg_torque_acados_get_nlp_opts(acados_ocp_capsule_leg);

    for (int i=0; i < FR_LEG_TORQUE_NBX0 ; i++) { idxbx_l[i] = i;}

    //Give default values to hyper-parameters:
    Wflatt.fill(0); 
    Wflatt_e.fill(0);
    W.setZero();
    Q.setZero();
    y_ref.setZero();

    //TODO: from parameter or something:
    tref_mh     << default_torque_target[0],default_torque_target[1],default_torque_target[2];

    update_reset_algorithm_quantities();

    //Initialize result:
    xtraj_l.fill(0);
    utraj_l.fill(0);

  }

  /// @brief This function updates the initial condition constraint. Should be called before each mpc evaluation
  /// @param x0 Initial condition.
  void leg_planner_update_constraints(double *x0){
    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "idxbx", idxbx_l.data());
    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, 0, "ubx", x0);
  }


  /// @brief This function updates the Weights and references in the mpc solver structure.
  /// Should be called only when changing phase
  /// TODO: remove y_ref overwrite
  void leg_plannet_update_weightsANDreference(){
    //1. Populate FlattenWeights with new values:
    for (int i=0;i<17 ; i++){
        Wflatt[i*17+i] = W[i] ;
    }

    for (int i=0;i<10 ; i++){
        Wflatt_e[i*10+i] = Q[i] ;
    }

    //2. Update the ocp struct
    for (int i = 0; i < N_l; i++){
        ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, i, "W", Wflatt.data());
    }
    ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, N_l, "W", Wflatt_e.data());
    
    //3. Set reference
    // y_ref.segment(0,3) = tref_mh; 
    // y_ref.segment(7,5) = q_interrupt; //TODO -> VALE INTERRUPT
    // y_ref.segment(0,3) << 0,0,1;
    // y_ref.segment(7,5) <<  1.4500,   -1.1000,    0.0816,    1.8500,    1.0829;

    for  (int i = 0; i < N_l; i++) {
        ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, i, "y_ref", y_ref.data());
    }
    ocp_nlp_cost_model_set(nlp_config_leg, nlp_dims_leg, nlp_in_leg, N_l, "y_ref", y_ref.tail(10).data());

  }


  /// @brief This function initializes the mpc solver
  void leg_planner_initialize_solution(Eigen::Matrix<double,FR_LEG_TORQUE_NBX,1> & x0){
    Eigen::Vector3d u0;
    u0.setZero();
    for (int i = 0; i < N_l; i++)
    {
        ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "x", x0.data());
        ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, i, "u", u0.data());
    }
    ocp_nlp_out_set(nlp_config_leg, nlp_dims_leg, nlp_out_leg, N_l, "x", x0.data());

  }

  public:
  /// @brief This function is called by the `leg_planner_timer` and: 
  /// 1. Reads the current state 
  /// 2. Evaluates the mpc
  /// 3. Resets the trajectory timer (even if allready stopped)
  /// @param  
  /// TODO: remove x0 - overwrite
  void leg_planner_update(const ros::TimerEvent&){
    leg_planner_update_();
  }

  void leg_planner_update_(){

    Eigen::Matrix<double,10,1> x0 ;

    //Reading from /joint_states-> so x0 is in rads
    x0 = leg_controllers[0]->get_full_state();
    x0.tail(5).setZero();
    x0[3] *= -1;  //hack for sings:
    
    leg_planner_update_constraints(x0.data());
    leg_planner_initialize_solution(x0);

    //update config
    ocp_nlp_solver_opts_set(nlp_config_leg, nlp_opts_leg, "rti_phase", &leg_planner_counter);

    //Solve: 
    int status = fr_leg_torque_acados_solve(acados_ocp_capsule_leg);

    trajectory_timer.stop(); //have the solution, so do not update the trajectory. I just have to get it

    double elapsed_time;
    //Get data:
    ocp_nlp_get(nlp_config_leg, nlp_solver_leg, "time_tot", &elapsed_time);
    elapsed_time = MIN(elapsed_time, 1e12);

    //get solution
    for (int ii = 0; ii <= nlp_dims_leg->N; ii++)
        ocp_nlp_out_get(nlp_config_leg, nlp_dims_leg, nlp_out_leg, ii, "x", &xtraj_l[ii*FR_LEG_TORQUE_NX]);
    for (int ii = 0; ii < nlp_dims_leg->N; ii++)
        ocp_nlp_out_get(nlp_config_leg, nlp_dims_leg, nlp_out_leg, ii, "u", &utraj_l[ii*FR_LEG_TORQUE_NU]);

    leg_planner_mpc_status.header.stamp = ros::Time::now();
    leg_planner_mpc_status.status = static_cast<uint8_t>(status);
    leg_planner_mpc_status.solution_time = 1e3*elapsed_time;
    leg_planner_status_pub.publish(leg_planner_mpc_status);

    // trajectory_timer.stop();
    trajectory_indexer = 0;
    trajectory_publish_();
    trajectory_timer.start();

    if (PRINT_LEG_PLANNER_MPC_RESULTS){
      printf("\n--- xtraj ---\n");
      d_print_exp_tran_mat( FR_LEG_TORQUE_NX, N_l+1, xtraj_l.data(), FR_LEG_TORQUE_NX);
      printf("\n--- utraj ---\n");
      d_print_exp_tran_mat( FR_LEG_TORQUE_NU, N_l, utraj_l.data(), FR_LEG_TORQUE_NU );
    }

  }

  // ===== RESET ALGO METHODS ========
  void check_mode(){
    int new_mode ; //[r: 0, p: 1, y: 2]

    //Impose particular modes when testing in testbed
    if (controller_config >=1 && controller_config <= 3){
      new_mode = controller_config - 1;
    }else{
      new_mode = resetting::get_max_u_id(utraj_b.data());
    }
    
    is_direction_positive = ( tref_mh[new_mode] > 0 ) ? true : false; 

    //1. Check for very low torque:
    // if (stabilization_mode && abs( utraj_b[new_mode] ) < TORQUE_MAGNITUDE_THRESH ){
    //   new_mode = controller_mode::stabilizing ; 
    // }
    if (stabilization_mode && ( quat_desired.angularDistance(quat_current)<ANGULAR_STABILIZATION_THRESH ) ){
      new_mode = controller_mode::stabilizing ;
    }

    if (new_mode != current_mode){
      current_mode =new_mode;

      if (new_mode == controller_mode::stabilizing ){
        trajectory_timer.stop();
        leg_planner_timer.stop(); 
        publish_phase(0,controller_mode::stabilizing, 0);
      
      }else{ //torque producing mode
        
        current_phase == phase::mid;
        update_reset_algorithm_quantities(); //update W,qref,resetting params and pass them to solver
        reset_leg_planner_publish();
        publish_phase(0,current_mode, current_phase);

      }

    }else{ //same mode
      if (current_mode !=controller_mode::stabilizing ){
        check_phase();
      }else{
        publish_phase(0,controller_mode::stabilizing, 0);
      }
    } 


  }  

  void check_phase(){
    using namespace resetting ;
    //1. read state from front_right leg
    auto q_fr = leg_controllers[0]->get_full_state().head(5) ; 
    q_fr[3] *= -1;

    //2. calculate error:
    Eigen::Matrix<double,5,1> q_error = q_fr - q_interrupt;  
    double error_norm = q_error.transpose() * W_interrupt * q_error ; 

    //3. Check if resetting
    if (error_norm < Th_intterupt){

      //progress to next phase.
      current_phase++;
      if (current_phase == NUMBER_OF_PHASES){ //return to phase 0: mid
        current_phase = phase::mid; 
      }

      //TODO: for every mode
      //4. Update the Weighting matrices, current setpoint, threshold 
      update_reset_algorithm_quantities(); //update weights-reference-resseting params and pass them to solver
      reset_leg_planner_publish(); //first run and resetting timers
    }

    publish_phase(error_norm,current_mode, current_phase);
  }

  void publish_phase(const double & error_norm, const int & mode, const int & phase ){
    phase_msg.error = error_norm;
    phase_msg.mode = mode;
    phase_msg.phase = static_cast<u_int8_t>( phase ) ;
    phase_msg.header.stamp  =ros::Time::now();
    phase_publisher.publish(phase_msg);
  }


  /// @brief This function calculates the updated weights/setpoints/resetting threshold based on the current phase, and then calls 
  /// `leg_plannet_update_weightsANDreference` to pass it to the mpc solver struct.
  void update_reset_algorithm_quantities(){
    std::array<int,4>  phase_order{0,1,2,3}; 
    if ( !is_direction_positive) { 
      phase_order[1] = 3;
      phase_order[3] = 1;
    }
    int resetting_param_id = phase_order[current_phase]; 
    ROS_INFO("resetting param is: %d",resetting_param_id);
    
    // RESET SELF QUANTITIES:
    using namespace resetting ;
      populate_diagonal(W_interrupt,*( yaw_W          [resetting_param_id] ) );
      populate_vector  (q_interrupt,*( yaw_setpoints  [resetting_param_id] ) );
      Th_intterupt = yaw_thresholds [resetting_param_id];

      contracting_ws(resetting_param_id); //quick hack
      
      double conv = 180./M_PI;
      ROS_INFO("[wbc controller]: Current setpoint is: [%.1f,%.1f,%.1f]",q_interrupt[0]*conv,q_interrupt[1]*conv,q_interrupt[3]*conv);
      ROS_INFO("[wbc controller]: Current phase is : %d",current_phase);
      ROS_INFO("[wbc controller]: Current threshold  is : %.1f",Th_intterupt);

      // MPC WEIGHTS AND REFERENCE
      qref = q_interrupt; 

      const std::array< double,12> * weights_ptr =  yaw_weights [current_phase] ;

      Wtrack[0] =   weights_ptr->at(0);
      Wtrack[1] =   weights_ptr->at(1);
      Wtrack[2] =   weights_ptr->at(2);

      Wu.fill( weights_ptr->at(3) ) ;

      Wstate[0] =   weights_ptr->at(4);
      Wstate.segment<4>(1).fill(    weights_ptr->at(5) );
      Wstate[5] =    weights_ptr->at(6) ;
      Wstate.segment<4>(6).fill(    weights_ptr->at(7) );

      Q[0] =   weights_ptr->at(8);
      Q.segment<4>(1).fill(    weights_ptr->at(9) );
      Q[5] =    weights_ptr->at(10) ;
      Q.segment<4>(6).fill(    weights_ptr->at(11) );

      leg_plannet_update_weightsANDreference();

  }

  void reset_leg_planner_publish(){
    leg_planner_timer.stop();
    leg_planner_update_(); //this starts the trajectory timer
    leg_planner_timer.start();

  }


  void contracting_ws(const int &  resetting_param){
    using namespace resetting ;
    Eigen::Matrix<double,5,1> yaw_CG_vec;
    populate_vector  (yaw_CG_vec,yaw_CG );

    double l = contracting_param[resetting_param];

    q_interrupt = q_interrupt*l + (1-l)*yaw_CG_vec; 
  }

  // ===== TRAJECTORY PLAYBACK FUNCTIONS ========
  private:
  void allocation(const Eigen::Vector3f & qd_fr){
    //initialize
    Eigen::Vector3f qd_rr;
    Eigen::Vector3f qd_fl;
    Eigen::Vector3f qd_rl;

    //matlab roll
    if (cancel_roll){ 
      qd_fl[0] = -qd_fr[0];
    } else { 
      qd_fl[0] = qd_fr[0] - 45;
    }

    //pitch-yaw
    if (pitch_mode){
      qd_fl[1] =  qd_fr[1];
      qd_fl[2] =  qd_fr[2];
    }else{ //yaw mode
      qd_fl[1] = -qd_fr[2]-ang_offset_yaw;
      qd_fl[2] = -(qd_fr[1]+ang_offset_yaw);
    }

    //Same side mimic:
    qd_rr[0] = -qd_fr[0]; //ok
    qd_rr[1] =  qd_fr[2];
    qd_rr[2] =  qd_fr[1];

    qd_rl[0] = -qd_fl[0]; //ok
    qd_rl[1] = -qd_fl[2];
    qd_rl[2] = -qd_fl[1];

    // hack to update signs:
    qd_fl[1] *= -1;
    qd_fl[2] *= -1;

    std::array< const Eigen::Vector3f *, 4> qd{&qd_fr,&qd_rr,&qd_fl,&qd_rl};
    // for (int i=0; i<4; i++){ 
    //   leg_controllers[i] -> setCommand( *qd[i] ); 
    // }
    leg_controllers[0] -> setCommand( *qd[0] );
  }

  void trajectory_publish(const ros::TimerEvent&){
    trajectory_publish_();
  }

  void trajectory_publish_(){
    int & i = trajectory_indexer ;
    i++;

    // Eigen::Vector3f pos(qMH_t[i],qHI_t[i],qHO_t[i]);
    double & qMH = xtraj_l[i*FR_LEG_TORQUE_NX    ];
    double & qHI = xtraj_l[i*FR_LEG_TORQUE_NX +1 ];
    double & qHO = xtraj_l[i*FR_LEG_TORQUE_NX +3 ];

    Eigen::Vector3f pos(qMH,qHI,qHO);
    // hack to update signs:
    pos[2] *= -1;
    allocation(pos*180/M_PI);

    if (DEBUG_TRAJECTORY_PUBLISHING){
      ROS_DEBUG_STREAM("Position is:" << pos*180/M_PI);
      ROS_DEBUG_STREAM("Qrref is:" << qref[0]*180/M_PI<<","<< qref[1]*180/M_PI<<","<< qref[3]*180/M_PI<<".");
      ROS_DEBUG_STREAM("Qinterupt is:" << q_interrupt[0]*180/M_PI<<","<< q_interrupt[1]*180/M_PI<<","<< q_interrupt[3]*180/M_PI<<".");
      ROS_DEBUG_STREAM("Q is"<<Q);
      ROS_DEBUG_STREAM("Wstate is"<<Wstate);
      ROS_DEBUG_STREAM("Wu is"<<Wu);
      ROS_DEBUG_STREAM("Wtrack is"<<Wtrack);
    }

    if (i == FR_LEG_TORQUE_N){
      ROS_DEBUG("reaching end of traj");
      i=0;
      trajectory_timer.stop();
    }
  }


  // ===== CONTROLLER ELEMENTS ========
  private:
  int trajectory_indexer = 0;
  ros::Timer trajectory_timer;
  std::array< std::unique_ptr< leg_controller>,4> leg_controllers;

  int body_planner_counter = 0;
  int leg_planner_counter = 0;

  int controller_config = base_joint::floating; //weather base is fixed or not -> how to read values
  bool stabilization_mode = true;
  
  std::array<double,3> default_torque_target;
  std::array<double,4> contracting_param{1,1,1,1};

  double ang_offset_roll  = 45;
  double ang_offset_pitch = 0;
  double ang_offset_yaw   = 0;

  // ===== BODY PLANNER PARAMETERS ========
  #pragma region
  // --- ros specific ---:
  ros::Publisher body_planner_status_pub;
  mpc_controller::mpc_status body_planner_mpc_status;
  ros::Timer body_planner_timer;
  ros::Subscriber olympus_current_pose_sub;
  ros::Subscriber olympus_desired_pose_sub;

  int N_b = SRBD_N; //Number of shooting intervals for body planner

  // --- states ---:
  Eigen::Quaterniond quat_current;
  Eigen::Vector3d    twist_current;
  Eigen::Quaterniond quat_desired;

  // --- trajectory ---:
  #define Nb 5
  #define NXb 7
  std::array<double, Nb*( NXb+1) > xtraj_b;   
  std::array<double, Nb*( NXb  ) > utraj_b; 

  // --- solver ---:
  SRBD_solver_capsule *acados_ocp_capsule_SRBD;
  ocp_nlp_config *nlp_config_SRBD ;
  ocp_nlp_dims *nlp_dims_SRBD ;
  ocp_nlp_in *nlp_in_SRBD ;
  ocp_nlp_out *nlp_out_SRBD ;
  ocp_nlp_solver *nlp_solver_SRBD ;
  void *nlp_opts_SRBD ;

  std::array<int,SRBD_NBX0> idxbx_b;

  #pragma endregion

  // ===== RESETTING PARAMETERS ========
  #pragma region
  Eigen::Matrix<double,5,1> q_interrupt;            //Variable holding the current setpoint
  Eigen::Matrix<double,5,5> W_interrupt;            //Variable holding the error weight
  double Th_intterupt = 0;                          //Variable holding the resetting threshold

  //Note -> a solution could be to use ptrs to not copy the data in W,q. But W,Q are in arrays, and should be always converted. 
  //Thus this more clean solution is chose,

  int current_mode  = controller_mode::stabilizing; //enum {r: 0, p: 1, y: 2, stabilization: 3} 
  int current_phase = phase::mid;                   //enum. Always starts from mid
  bool is_direction_positive = true; 

  bool cancel_roll = true;                          //TODO: maybe do not use
  bool pitch_mode  = false;

  ros::Publisher phase_publisher;
  mpc_controller::phase phase_msg;
  #pragma endregion

  // ===== LEG PLANNER PARAMETERS ========
  #pragma region
  ros::Publisher leg_planner_status_pub;
  mpc_controller::mpc_status leg_planner_mpc_status;
  ros::Timer leg_planner_timer;
  int N_l = FR_LEG_TORQUE_N; //Number of shooting intervals for torque planner

  // ---trajectory---:
  std::array<double, FR_LEG_TORQUE_NX * (FR_LEG_TORQUE_N+1)> xtraj_l;
  std::array<double, FR_LEG_TORQUE_NU * (FR_LEG_TORQUE_N  )> utraj_l;

  #define FR_LEG_TORQUE_NYREF 17
  #define FR_LEG_TORQUE_NYREF_E 10

  // Leg Planner Weights variables:
  Eigen::Matrix<double,FR_LEG_TORQUE_NYREF  ,1> W;
  Eigen::Matrix<double,FR_LEG_TORQUE_NYREF_E,1> Q;

  //Aliases for Weight parts
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,3,1>  Wtrack   = W.segment<3>(0); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,2,1>  Wu       = W.segment<2>(3); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,2,1>  Wclosure = W.segment<2>(5); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,10,1> Wstate   = W.segment<10>(7); 

  // Flatten diagonal arrays:
  std::array<double,17*17> Wflatt;   //Class Variable -> so i do not fill with 0 all the time
  std::array<double,10*10> Wflatt_e; //Class Variable

  // Leg Planner Weights variables:
  Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1> y_ref ;

  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,3,1>  tref_mh     = y_ref.segment<3>(0); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,2,1>  input_ref   = y_ref.segment<2>(3); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,2,1>  closure_ref = y_ref.segment<2>(5); 
  Eigen::Block< Eigen::Matrix<double,FR_LEG_TORQUE_NYREF,1>,5,1>  qref        = y_ref.segment<5>(7); 
  
  // --- solver ---:
  fr_leg_torque_solver_capsule *acados_ocp_capsule_leg;
  ocp_nlp_config *nlp_config_leg ;
  ocp_nlp_dims *nlp_dims_leg ;
  ocp_nlp_in *nlp_in_leg ;
  ocp_nlp_out *nlp_out_leg ;
  ocp_nlp_solver *nlp_solver_leg ;
  void *nlp_opts_leg ;

  std::array<int,FR_LEG_TORQUE_NBX0> idxbx_l;


  #pragma endregion
  
};

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;

double loop_freq = 1000 ;
ros::Rate loop_rate(loop_freq); //HZ
#pragma endregion

whole_body_controller wbc;


while(ros::ok()){
  wbc.check_mode();
  ros::spinOnce(); // Process any callbacks
  loop_rate.sleep();
}
return 0;
}