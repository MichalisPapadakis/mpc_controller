#include <ros/ros.h>
//link with generated library:
//with ACADOS_INCLUDE_DIRS
// #include "acados/utils/types.h"
#include "eigen3/Eigen/Dense"

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "c_generated_code/acados_solver_fr_leg_pos.h" 
// #include "acados_solver_fr_leg_pos.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     FR_LEG_POS_NX //used
// #define NZ     FR_LEG_POS_NZ
#define NU     FR_LEG_POS_NU //used
// #define NP     FR_LEG_POS_NP
// #define NBX    FR_LEG_POS_NBX
#define NBX0   FR_LEG_POS_NBX0 //used
// #define NBU    FR_LEG_POS_NBU
// #define NSBX   FR_LEG_POS_NSBX
// #define NSBU   FR_LEG_POS_NSBU
// #define NSH    FR_LEG_POS_NSH
// #define NSG    FR_LEG_POS_NSG
// #define NSPHI  FR_LEG_POS_NSPHI
// #define NSHN   FR_LEG_POS_NSHN
// #define NSGN   FR_LEG_POS_NSGN
// #define NSPHIN FR_LEG_POS_NSPHIN
// #define NSBXN  FR_LEG_POS_NSBXN
// #define NS     FR_LEG_POS_NS
// #define NSN    FR_LEG_POS_NSN
// #define NG     FR_LEG_POS_NG
// #define NBXN   FR_LEG_POS_NBXN
// #define NGN    FR_LEG_POS_NGN
// #define NY0    FR_LEG_POS_NY0
// #define NY     FR_LEG_POS_NY
// #define NYN    FR_LEG_POS_NYN
// #define NH     FR_LEG_POS_NH
// #define NPHI   FR_LEG_POS_NPHI
// #define NHN    FR_LEG_POS_NHN
// #define NPHIN  FR_LEG_POS_NPHIN
// #define NR     FR_LEG_POS_NR

int main(int argc, char **argv) {

// Ros: node SETUP
#pragma region
ros::init(argc, argv, "test_acados");
ros::NodeHandle nh;

// double loop_freq = 100 ;
// ros::Rate loop_rate(loop_freq); //HZ
// #pragma endregion

// while(ros::ok()){
//   ros::spinOnce(); // Process any callbacks
//   loop_rate.sleep();
// }

fr_leg_pos_solver_capsule *acados_ocp_capsule = fr_leg_pos_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = FR_LEG_POS_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = fr_leg_pos_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("fr_leg_pos_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = fr_leg_pos_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = fr_leg_pos_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = fr_leg_pos_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = fr_leg_pos_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = fr_leg_pos_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = fr_leg_pos_acados_get_nlp_opts(acados_ocp_capsule);


    //Set initial conditions:
    #pragma region 
    std::array<int,NBX0> idxbx;
    for (int id=0; id< NBX0 ; id++){ idxbx[id]=id; }
    Eigen::Matrix<double,NBX0,1> x0 ;
    x0.setZero();   
    Eigen::Vector3d u0;
    u0.setZero();

    //Set target:
    Eigen::Matrix<double,NBX0,1> xf ;
    Eigen::Matrix<double,5,1>  qref; 
    qref <<  0,1.8000,-1.1703,-0.7000,0.4087;    
    xf.setZero();
    xf.head(5) = qref;

    //Set constraints 
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0.data());

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", xf.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", xf.data());
    #pragma endregion

    // prepare evaluation
    // int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    std::array<double,NX*(FR_LEG_POS_N+1)> xtraj;
    std::array<double,NU* FR_LEG_POS_N   > utraj;


    // solve ocp in loop
    int rti_phase = 0;

    // for (int ii = 0; ii < NTIMINGS; ii++)
    // {

    // initialize solution
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0.data());
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0.data());
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0.data());

    //update config
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);

    //Solve: 
    status = fr_leg_pos_acados_solve(acados_ocp_capsule);

    //Get data:
    ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);

    // }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    //Actual printing
    #pragma region


    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj.data(), NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj.data(), NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    // printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("fr_leg_pos_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("fr_leg_pos_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    fr_leg_pos_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    // printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
        //    sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    #pragma endregion    


    // free solver -> After while
    status = fr_leg_pos_acados_free(acados_ocp_capsule);
    if (status) {
        printf("fr_leg_pos_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = fr_leg_pos_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("fr_leg_pos_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
return 0;
}