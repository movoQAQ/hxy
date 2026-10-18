/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef ACADOS_SOLVER_omni_mpc_H_
#define ACADOS_SOLVER_omni_mpc_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define OMNI_MPC_NX     4
#define OMNI_MPC_NZ     0
#define OMNI_MPC_NU     2
#define OMNI_MPC_NP     540
#define OMNI_MPC_NP_GLOBAL     0
#define OMNI_MPC_NBX    0
#define OMNI_MPC_NBX0   4
#define OMNI_MPC_NBU    0
#define OMNI_MPC_NSBX   0
#define OMNI_MPC_NSBU   0
#define OMNI_MPC_NSH    0
#define OMNI_MPC_NSH0   0
#define OMNI_MPC_NSG    0
#define OMNI_MPC_NSPHI  0
#define OMNI_MPC_NSHN   0
#define OMNI_MPC_NSGN   0
#define OMNI_MPC_NSPHIN 0
#define OMNI_MPC_NSPHI0 0
#define OMNI_MPC_NSBXN  0
#define OMNI_MPC_NS     0
#define OMNI_MPC_NS0    0
#define OMNI_MPC_NSN    0
#define OMNI_MPC_NG     0
#define OMNI_MPC_NBXN   0
#define OMNI_MPC_NGN    0
#define OMNI_MPC_NY0    0
#define OMNI_MPC_NY     0
#define OMNI_MPC_NYN    0
#define OMNI_MPC_N      30
#define OMNI_MPC_NH     137
#define OMNI_MPC_NHN    132
#define OMNI_MPC_NH0    137
#define OMNI_MPC_NPHI0  0
#define OMNI_MPC_NPHI   0
#define OMNI_MPC_NPHIN  0
#define OMNI_MPC_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct omni_mpc_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *discr_dyn_phi_fun;
    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt;



    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt_hess;


    // cost

    external_function_external_param_casadi *ext_cost_fun;
    external_function_external_param_casadi *ext_cost_fun_jac;
    external_function_external_param_casadi *ext_cost_fun_jac_hess;






    external_function_external_param_casadi ext_cost_0_fun;
    external_function_external_param_casadi ext_cost_0_fun_jac;
    external_function_external_param_casadi ext_cost_0_fun_jac_hess;





    external_function_external_param_casadi ext_cost_e_fun;
    external_function_external_param_casadi ext_cost_e_fun_jac;
    external_function_external_param_casadi ext_cost_e_fun_jac_hess;




    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;
    external_function_external_param_casadi *nl_constr_h_fun_jac_hess;






    external_function_external_param_casadi nl_constr_h_0_fun_jac;
    external_function_external_param_casadi nl_constr_h_0_fun;
    external_function_external_param_casadi nl_constr_h_0_fun_jac_hess;






    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;
    external_function_external_param_casadi nl_constr_h_e_fun_jac_hess;




} omni_mpc_solver_capsule;

ACADOS_SYMBOL_EXPORT omni_mpc_solver_capsule * omni_mpc_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_free_capsule(omni_mpc_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int omni_mpc_acados_create(omni_mpc_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int omni_mpc_acados_reset(omni_mpc_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar);

/**
 * Generic version of omni_mpc_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_create_with_discretization(omni_mpc_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_update_time_steps(omni_mpc_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_update_qp_solver_cond_N(omni_mpc_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_update_params(omni_mpc_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_update_params_sparse(omni_mpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_set_p_global_and_precompute_dependencies(omni_mpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int omni_mpc_acados_solve(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_setup_qp_matrices_and_factorize(omni_mpc_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int omni_mpc_acados_free(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void omni_mpc_acados_print_stats(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int omni_mpc_acados_custom_update(omni_mpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *omni_mpc_acados_get_nlp_in(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *omni_mpc_acados_get_nlp_out(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *omni_mpc_acados_get_sens_out(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *omni_mpc_acados_get_nlp_solver(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *omni_mpc_acados_get_nlp_config(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *omni_mpc_acados_get_nlp_opts(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *omni_mpc_acados_get_nlp_dims(omni_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *omni_mpc_acados_get_nlp_plan(omni_mpc_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_omni_mpc_H_
