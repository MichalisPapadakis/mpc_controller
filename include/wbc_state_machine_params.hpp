#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>

enum controller_mode { roll,pitch,yaw,stabilizing };
enum phase {mid, reset, ext, torque };




namespace resetting{
#define torque_magn_threshold 0.2 
constexpr int NUMBER_OF_PHASES = 4;

//u is saved in a large array
//To avoid passing the define, we use this
//TODO: maybe implement with array, if sizes of mpcs are defined here
inline int get_max_u_id(const double*  u ){
  int id_max;
  id_max = ( abs( u[0]      )  > abs( u[1] ) ) ? 0      : 1 ; 
  return   ( abs( u[id_max] )  > abs( u[2] ) ) ? id_max : 1 ;
}

void populate_diagonal(Eigen::Matrix<double,5,5> & vec, const std::array<double,5> arr){
  vec.diagonal() << arr[0],arr[1],arr[2],arr[3],arr[4];
}

void populate_vector(Eigen::Matrix<double,5,1> & vec, const std::array<double,5> arr){
  vec << arr[0],arr[1],arr[2],arr[3],arr[4];
}

// ===== SETPOINTS ======== 

// ROLL:
constexpr std::array<double, 5> roll_pos   {  1.85,        0,        0,        0,        0 };
constexpr std::array<double, 5> roll_neg   {  -1.2,   0.3500,  -0.5542,   0.3500,   0.5605 };
constexpr std::array<double, 5> roll_ext   {     0,        0,        0,        0,        0 };
constexpr std::array<double, 5> roll_mid   {     0,   1.3500,  -2.0695,   1.5000,   2.1095 };

constexpr std::array<double, 5> roll_W_mid {     1,     5,        0,        5,        0 };
constexpr std::array<double, 5> roll_W_res {     1,     0,        0,        0,        0 };
constexpr std::array<double, 5> roll_W_ext {     1,     0,        0,        0,        0 };
constexpr std::array<double, 5> roll_W_tor {     1,     0,        0,        0,        0 };

#define roll_Th_mid 0.25
#define roll_Th_res 0.1
#define roll_Th_ext 0.1
#define roll_Th_tor 0.25

constexpr std::array<const std::array<double, 5> * , 4> roll_setpoints  {  &roll_mid,   &roll_neg,   &roll_ext,   &roll_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> roll_W          {&roll_W_mid, &roll_W_res, &roll_W_ext, &roll_W_tor } ;
constexpr std::array<double  , 4>                       roll_thresholds {roll_Th_mid, roll_Th_res, roll_Th_ext, roll_Th_tor } ;

// PITCH:
constexpr std::array<double, 5> pitch_pos   {      0,  2.0900,  -1.5456,  -0.3330,   1.0078 };
constexpr std::array<double, 5> pitch_neg   {      0, -1.0000,  -1.3412,   3.5000,   1.8464 };
constexpr std::array<double, 5> pitch_ext   {      0,       0,        0,        0,        0 };
constexpr std::array<double, 5> pitch_mid   { M_PI/2,       0,        0,        0,        0 };

constexpr std::array<double, 5> pitch_W_mid {   0.5,     1,        0,        1,        0 };
constexpr std::array<double, 5> pitch_W_res {   0.5,     1,        0,        1,        0 };
constexpr std::array<double, 5> pitch_W_ext {     0,   0.4,        0,        2,        0 };
constexpr std::array<double, 5> pitch_W_tor {     0,   0.4,        0,        2,        0 };

#define pitch_Th_mid 10
#define pitch_Th_res 2
#define pitch_Th_ext 7.5
#define pitch_Th_tor 0.2
#define pitch_inject_1_3 -2 //Inject this in the matrix in the pitch mode for ext-tor phases


constexpr std::array<const std::array<double, 5> * , 4> pitch_setpoints  {  &pitch_mid,   &pitch_neg,   &pitch_ext,   &pitch_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> pitch_W          {&pitch_W_mid, &pitch_W_res, &pitch_W_ext, &pitch_W_tor } ;
constexpr std::array<double  , 4>                       pitch_thresholds {pitch_Th_mid, pitch_Th_res, pitch_Th_ext, pitch_Th_tor } ;


//YAW: 
constexpr std::array<double,5> yaw_pos {1.4500,   -1.1000,    0.0816,    1.8500,    1.0829 } ;
constexpr std::array<double,5> yaw_neg {1.4700,    1.0400,   -0.7702,   -0.6000,   -0.1164 } ;
constexpr std::array<double,5> yaw_mid {     0,    0.6800,   -1.2675,    1.0000,    1.3578 } ;
constexpr std::array<double,5> yaw_ext { 1.300,    0.1500,   -0.4166,    0.4700 ,   0.5689 } ;


constexpr std::array<double,5> yaw_W_mid {   1,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_res { 0.5,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_ext {   1,1,0,1,0 } ;
constexpr std::array<double,5> yaw_W_tor { 0.4,1,0,1,0 } ;

#define yaw_Th_mid 0.5
#define yaw_Th_res 0.2
#define yaw_Th_ext 0.15
#define yaw_Th_tor 0.5

constexpr std::array<const std::array<double, 5> * , 4> yaw_setpoints  {  &yaw_mid,   &yaw_neg,   &yaw_ext,   &yaw_pos } ;
constexpr std::array<const std::array<double, 5> * , 4> yaw_W          {&yaw_W_mid, &yaw_W_res, &yaw_W_ext, &yaw_W_tor } ;
constexpr std::array<double  , 4>                       yaw_thresholds {yaw_Th_mid, yaw_Th_res, yaw_Th_ext, yaw_Th_tor } ;



//TODO: organize roll,yaw,pitch setpoints and handle stabilization

};
