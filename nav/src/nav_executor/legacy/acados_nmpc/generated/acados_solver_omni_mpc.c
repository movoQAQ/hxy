/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h> // memcpy
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "omni_mpc_model/omni_mpc_model.h"


#include "omni_mpc_constraints/omni_mpc_constraints.h"
#include "omni_mpc_cost/omni_mpc_cost.h"



#include "acados_solver_omni_mpc.h"

#define NX     OMNI_MPC_NX
#define NZ     OMNI_MPC_NZ
#define NU     OMNI_MPC_NU
#define NP     OMNI_MPC_NP
#define NP_GLOBAL     OMNI_MPC_NP_GLOBAL
#define NY0    OMNI_MPC_NY0
#define NY     OMNI_MPC_NY
#define NYN    OMNI_MPC_NYN

#define NBX    OMNI_MPC_NBX
#define NBX0   OMNI_MPC_NBX0
#define NBU    OMNI_MPC_NBU
#define NG     OMNI_MPC_NG
#define NBXN   OMNI_MPC_NBXN
#define NGN    OMNI_MPC_NGN

#define NH     OMNI_MPC_NH
#define NHN    OMNI_MPC_NHN
#define NH0    OMNI_MPC_NH0
#define NPHI   OMNI_MPC_NPHI
#define NPHIN  OMNI_MPC_NPHIN
#define NPHI0  OMNI_MPC_NPHI0
#define NR     OMNI_MPC_NR

#define NS     OMNI_MPC_NS
#define NS0    OMNI_MPC_NS0
#define NSN    OMNI_MPC_NSN

#define NSBX   OMNI_MPC_NSBX
#define NSBU   OMNI_MPC_NSBU
#define NSH0   OMNI_MPC_NSH0
#define NSH    OMNI_MPC_NSH
#define NSHN   OMNI_MPC_NSHN
#define NSG    OMNI_MPC_NSG
#define NSPHI0 OMNI_MPC_NSPHI0
#define NSPHI  OMNI_MPC_NSPHI
#define NSPHIN OMNI_MPC_NSPHIN
#define NSGN   OMNI_MPC_NSGN
#define NSBXN  OMNI_MPC_NSBXN





// ** solver data **

omni_mpc_solver_capsule * omni_mpc_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(omni_mpc_solver_capsule));
    omni_mpc_solver_capsule *capsule = (omni_mpc_solver_capsule *) capsule_mem;

    return capsule;
}


int omni_mpc_acados_free_capsule(omni_mpc_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int omni_mpc_acados_create(omni_mpc_solver_capsule* capsule)
{
    int N_shooting_intervals = OMNI_MPC_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return omni_mpc_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int omni_mpc_acados_update_time_steps(omni_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "omni_mpc_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for omni_mpc_acados_create: step 1
 */
void omni_mpc_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = MIRROR;

    nlp_solver_plan->globalization = MERIT_BACKTRACKING;
}


static ocp_nlp_dims* omni_mpc_acados_create_setup_dimensions(omni_mpc_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 4;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for omni_mpc_acados_create: step 3
 */
void omni_mpc_acados_create_setup_functions(omni_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        MAP_CASADI_FNC(nl_constr_h_0_fun_jac, omni_mpc_constr_h_0_fun_jac_uxt_zt);
        MAP_CASADI_FNC(nl_constr_h_0_fun, omni_mpc_constr_h_0_fun);
        MAP_CASADI_FNC(nl_constr_h_0_fun_jac_hess, omni_mpc_constr_h_0_fun_jac_uxt_zt_hess);
        
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], omni_mpc_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], omni_mpc_constr_h_fun);
        }
        capsule->nl_constr_h_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac_hess[i], omni_mpc_constr_h_fun_jac_uxt_zt_hess);
        }
    
        // external cost
        MAP_CASADI_FNC(ext_cost_0_fun, omni_mpc_cost_ext_cost_0_fun);
        MAP_CASADI_FNC(ext_cost_0_fun_jac, omni_mpc_cost_ext_cost_0_fun_jac);
        MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, omni_mpc_cost_ext_cost_0_fun_jac_hess);



    
        // discrete dynamics
        capsule->discr_dyn_phi_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun[i], omni_mpc_dyn_disc_phi_fun);
        }

        capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], omni_mpc_dyn_disc_phi_fun_jac);
        }

    

    

    
        capsule->discr_dyn_phi_fun_jac_ut_xt_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt_hess[i], omni_mpc_dyn_disc_phi_fun_jac_hess);
        }
        // external cost
        capsule->ext_cost_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun[i], omni_mpc_cost_ext_cost_fun);
        }

        capsule->ext_cost_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun_jac[i], omni_mpc_cost_ext_cost_fun_jac);
        }

        capsule->ext_cost_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], omni_mpc_cost_ext_cost_fun_jac_hess);
        }

        


        


        
    } // N > 0
    MAP_CASADI_FNC(nl_constr_h_e_fun_jac, omni_mpc_constr_h_e_fun_jac_uxt_zt);
    MAP_CASADI_FNC(nl_constr_h_e_fun, omni_mpc_constr_h_e_fun);
    MAP_CASADI_FNC(nl_constr_h_e_fun_jac_hess, omni_mpc_constr_h_e_fun_jac_uxt_zt_hess);
    
    
    
    
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, omni_mpc_cost_ext_cost_e_fun);

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, omni_mpc_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, omni_mpc_cost_ext_cost_e_fun_jac_hess);

    // external cost - jacobian wrt params
    

    

    

#undef MAP_CASADI_FNC
}


/**
 * Internal function for omni_mpc_acados_create: step 5
 */
void omni_mpc_acados_create_set_default_parameters(omni_mpc_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;

    // initialize parameters to initial value
    
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        omni_mpc_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for omni_mpc_acados_create: step 5
 */
void omni_mpc_acados_create_setup_nlp_in_numerical_values(omni_mpc_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        omni_mpc_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.05;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.05;
        cost_scaling[1] = 0.05;
        cost_scaling[2] = 0.05;
        cost_scaling[3] = 0.05;
        cost_scaling[4] = 0.05;
        cost_scaling[5] = 0.05;
        cost_scaling[6] = 0.05;
        cost_scaling[7] = 0.05;
        cost_scaling[8] = 0.05;
        cost_scaling[9] = 0.05;
        cost_scaling[10] = 0.05;
        cost_scaling[11] = 0.05;
        cost_scaling[12] = 0.05;
        cost_scaling[13] = 0.05;
        cost_scaling[14] = 0.05;
        cost_scaling[15] = 0.05;
        cost_scaling[16] = 0.05;
        cost_scaling[17] = 0.05;
        cost_scaling[18] = 0.05;
        cost_scaling[19] = 0.05;
        cost_scaling[20] = 0.05;
        cost_scaling[21] = 0.05;
        cost_scaling[22] = 0.05;
        cost_scaling[23] = 0.05;
        cost_scaling[24] = 0.05;
        cost_scaling[25] = 0.05;
        cost_scaling[26] = 0.05;
        cost_scaling[27] = 0.05;
        cost_scaling[28] = 0.05;
        cost_scaling[29] = 0.05;
        cost_scaling[30] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Cost ****/






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(4 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);



    // set up nonlinear constraints for last stage
    double* luh_0 = calloc(2*NH0, sizeof(double));
    double* lh_0 = luh_0;
    double* uh_0 = luh_0 + NH0;
    uh_0[0] = 10000000000;
    uh_0[1] = 10000000000;
    uh_0[2] = 10000000000;
    uh_0[3] = 10000000000;
    uh_0[4] = 10000000000;
    uh_0[5] = 10000000000;
    uh_0[6] = 10000000000;
    uh_0[7] = 10000000000;
    uh_0[8] = 10000000000;
    uh_0[9] = 10000000000;
    uh_0[10] = 10000000000;
    uh_0[11] = 10000000000;
    uh_0[12] = 10000000000;
    uh_0[13] = 10000000000;
    uh_0[14] = 10000000000;
    uh_0[15] = 10000000000;
    uh_0[16] = 10000000000;
    uh_0[17] = 10000000000;
    uh_0[18] = 10000000000;
    uh_0[19] = 10000000000;
    uh_0[20] = 10000000000;
    uh_0[21] = 10000000000;
    uh_0[22] = 10000000000;
    uh_0[23] = 10000000000;
    uh_0[24] = 10000000000;
    uh_0[25] = 10000000000;
    uh_0[26] = 10000000000;
    uh_0[27] = 10000000000;
    uh_0[28] = 10000000000;
    uh_0[29] = 10000000000;
    uh_0[30] = 10000000000;
    uh_0[31] = 10000000000;
    uh_0[32] = 10000000000;
    uh_0[33] = 10000000000;
    uh_0[34] = 10000000000;
    uh_0[35] = 10000000000;
    uh_0[36] = 10000000000;
    uh_0[37] = 10000000000;
    uh_0[38] = 10000000000;
    uh_0[39] = 10000000000;
    uh_0[40] = 10000000000;
    uh_0[41] = 10000000000;
    uh_0[42] = 10000000000;
    uh_0[43] = 10000000000;
    uh_0[44] = 10000000000;
    uh_0[45] = 10000000000;
    uh_0[46] = 10000000000;
    uh_0[47] = 10000000000;
    uh_0[48] = 10000000000;
    uh_0[49] = 10000000000;
    uh_0[50] = 10000000000;
    uh_0[51] = 10000000000;
    uh_0[52] = 10000000000;
    uh_0[53] = 10000000000;
    uh_0[54] = 10000000000;
    uh_0[55] = 10000000000;
    uh_0[56] = 10000000000;
    uh_0[57] = 10000000000;
    uh_0[58] = 10000000000;
    uh_0[59] = 10000000000;
    uh_0[60] = 10000000000;
    uh_0[61] = 10000000000;
    uh_0[62] = 10000000000;
    uh_0[63] = 10000000000;
    uh_0[64] = 10000000000;
    uh_0[65] = 10000000000;
    uh_0[66] = 10000000000;
    uh_0[67] = 10000000000;
    uh_0[68] = 10000000000;
    uh_0[69] = 10000000000;
    uh_0[70] = 10000000000;
    uh_0[71] = 10000000000;
    uh_0[72] = 10000000000;
    uh_0[73] = 10000000000;
    uh_0[74] = 10000000000;
    uh_0[75] = 10000000000;
    uh_0[76] = 10000000000;
    uh_0[77] = 10000000000;
    uh_0[78] = 10000000000;
    uh_0[79] = 10000000000;
    uh_0[80] = 10000000000;
    uh_0[81] = 10000000000;
    uh_0[82] = 10000000000;
    uh_0[83] = 10000000000;
    uh_0[84] = 10000000000;
    uh_0[85] = 10000000000;
    uh_0[86] = 10000000000;
    uh_0[87] = 10000000000;
    uh_0[88] = 10000000000;
    uh_0[89] = 10000000000;
    uh_0[90] = 10000000000;
    uh_0[91] = 10000000000;
    uh_0[92] = 10000000000;
    uh_0[93] = 10000000000;
    uh_0[94] = 10000000000;
    uh_0[95] = 10000000000;
    uh_0[96] = 10000000000;
    uh_0[97] = 10000000000;
    uh_0[98] = 10000000000;
    uh_0[99] = 10000000000;
    uh_0[100] = 10000000000;
    uh_0[101] = 10000000000;
    uh_0[102] = 10000000000;
    uh_0[103] = 10000000000;
    uh_0[104] = 10000000000;
    uh_0[105] = 10000000000;
    uh_0[106] = 10000000000;
    uh_0[107] = 10000000000;
    uh_0[108] = 10000000000;
    uh_0[109] = 10000000000;
    uh_0[110] = 10000000000;
    uh_0[111] = 10000000000;
    uh_0[112] = 10000000000;
    uh_0[113] = 10000000000;
    uh_0[114] = 10000000000;
    uh_0[115] = 10000000000;
    uh_0[116] = 10000000000;
    uh_0[117] = 10000000000;
    uh_0[118] = 10000000000;
    uh_0[119] = 10000000000;
    uh_0[120] = 10000000000;
    uh_0[121] = 10000000000;
    uh_0[122] = 10000000000;
    uh_0[123] = 10000000000;
    uh_0[124] = 10000000000;
    uh_0[125] = 10000000000;
    uh_0[126] = 10000000000;
    uh_0[127] = 10000000000;
    uh_0[128] = 10000000000;
    uh_0[129] = 10000000000;
    uh_0[130] = 10000000000;
    uh_0[131] = 10000000000;
    uh_0[132] = 10000000000;
    uh_0[133] = 10000000000;
    uh_0[134] = 10000000000;
    uh_0[135] = 10000000000;
    uh_0[136] = 10000000000;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lh", lh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "uh", uh_0);
    free(luh_0);









    /* constraints that are the same for initial and intermediate */






    /* Path constraints */



    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    uh[0] = 10000000000;
    uh[1] = 10000000000;
    uh[2] = 10000000000;
    uh[3] = 10000000000;
    uh[4] = 10000000000;
    uh[5] = 10000000000;
    uh[6] = 10000000000;
    uh[7] = 10000000000;
    uh[8] = 10000000000;
    uh[9] = 10000000000;
    uh[10] = 10000000000;
    uh[11] = 10000000000;
    uh[12] = 10000000000;
    uh[13] = 10000000000;
    uh[14] = 10000000000;
    uh[15] = 10000000000;
    uh[16] = 10000000000;
    uh[17] = 10000000000;
    uh[18] = 10000000000;
    uh[19] = 10000000000;
    uh[20] = 10000000000;
    uh[21] = 10000000000;
    uh[22] = 10000000000;
    uh[23] = 10000000000;
    uh[24] = 10000000000;
    uh[25] = 10000000000;
    uh[26] = 10000000000;
    uh[27] = 10000000000;
    uh[28] = 10000000000;
    uh[29] = 10000000000;
    uh[30] = 10000000000;
    uh[31] = 10000000000;
    uh[32] = 10000000000;
    uh[33] = 10000000000;
    uh[34] = 10000000000;
    uh[35] = 10000000000;
    uh[36] = 10000000000;
    uh[37] = 10000000000;
    uh[38] = 10000000000;
    uh[39] = 10000000000;
    uh[40] = 10000000000;
    uh[41] = 10000000000;
    uh[42] = 10000000000;
    uh[43] = 10000000000;
    uh[44] = 10000000000;
    uh[45] = 10000000000;
    uh[46] = 10000000000;
    uh[47] = 10000000000;
    uh[48] = 10000000000;
    uh[49] = 10000000000;
    uh[50] = 10000000000;
    uh[51] = 10000000000;
    uh[52] = 10000000000;
    uh[53] = 10000000000;
    uh[54] = 10000000000;
    uh[55] = 10000000000;
    uh[56] = 10000000000;
    uh[57] = 10000000000;
    uh[58] = 10000000000;
    uh[59] = 10000000000;
    uh[60] = 10000000000;
    uh[61] = 10000000000;
    uh[62] = 10000000000;
    uh[63] = 10000000000;
    uh[64] = 10000000000;
    uh[65] = 10000000000;
    uh[66] = 10000000000;
    uh[67] = 10000000000;
    uh[68] = 10000000000;
    uh[69] = 10000000000;
    uh[70] = 10000000000;
    uh[71] = 10000000000;
    uh[72] = 10000000000;
    uh[73] = 10000000000;
    uh[74] = 10000000000;
    uh[75] = 10000000000;
    uh[76] = 10000000000;
    uh[77] = 10000000000;
    uh[78] = 10000000000;
    uh[79] = 10000000000;
    uh[80] = 10000000000;
    uh[81] = 10000000000;
    uh[82] = 10000000000;
    uh[83] = 10000000000;
    uh[84] = 10000000000;
    uh[85] = 10000000000;
    uh[86] = 10000000000;
    uh[87] = 10000000000;
    uh[88] = 10000000000;
    uh[89] = 10000000000;
    uh[90] = 10000000000;
    uh[91] = 10000000000;
    uh[92] = 10000000000;
    uh[93] = 10000000000;
    uh[94] = 10000000000;
    uh[95] = 10000000000;
    uh[96] = 10000000000;
    uh[97] = 10000000000;
    uh[98] = 10000000000;
    uh[99] = 10000000000;
    uh[100] = 10000000000;
    uh[101] = 10000000000;
    uh[102] = 10000000000;
    uh[103] = 10000000000;
    uh[104] = 10000000000;
    uh[105] = 10000000000;
    uh[106] = 10000000000;
    uh[107] = 10000000000;
    uh[108] = 10000000000;
    uh[109] = 10000000000;
    uh[110] = 10000000000;
    uh[111] = 10000000000;
    uh[112] = 10000000000;
    uh[113] = 10000000000;
    uh[114] = 10000000000;
    uh[115] = 10000000000;
    uh[116] = 10000000000;
    uh[117] = 10000000000;
    uh[118] = 10000000000;
    uh[119] = 10000000000;
    uh[120] = 10000000000;
    uh[121] = 10000000000;
    uh[122] = 10000000000;
    uh[123] = 10000000000;
    uh[124] = 10000000000;
    uh[125] = 10000000000;
    uh[126] = 10000000000;
    uh[127] = 10000000000;
    uh[128] = 10000000000;
    uh[129] = 10000000000;
    uh[130] = 10000000000;
    uh[131] = 10000000000;
    uh[132] = 10000000000;
    uh[133] = 10000000000;
    uh[134] = 10000000000;
    uh[135] = 10000000000;
    uh[136] = 10000000000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
    }
    free(luh);











    /* terminal constraints */





    // set up nonlinear constraints for last stage
    double* luh_e = calloc(2*NHN, sizeof(double));
    double* lh_e = luh_e;
    double* uh_e = luh_e + NHN;
    uh_e[0] = 10000000000;
    uh_e[1] = 10000000000;
    uh_e[2] = 10000000000;
    uh_e[3] = 10000000000;
    uh_e[4] = 10000000000;
    uh_e[5] = 10000000000;
    uh_e[6] = 10000000000;
    uh_e[7] = 10000000000;
    uh_e[8] = 10000000000;
    uh_e[9] = 10000000000;
    uh_e[10] = 10000000000;
    uh_e[11] = 10000000000;
    uh_e[12] = 10000000000;
    uh_e[13] = 10000000000;
    uh_e[14] = 10000000000;
    uh_e[15] = 10000000000;
    uh_e[16] = 10000000000;
    uh_e[17] = 10000000000;
    uh_e[18] = 10000000000;
    uh_e[19] = 10000000000;
    uh_e[20] = 10000000000;
    uh_e[21] = 10000000000;
    uh_e[22] = 10000000000;
    uh_e[23] = 10000000000;
    uh_e[24] = 10000000000;
    uh_e[25] = 10000000000;
    uh_e[26] = 10000000000;
    uh_e[27] = 10000000000;
    uh_e[28] = 10000000000;
    uh_e[29] = 10000000000;
    uh_e[30] = 10000000000;
    uh_e[31] = 10000000000;
    uh_e[32] = 10000000000;
    uh_e[33] = 10000000000;
    uh_e[34] = 10000000000;
    uh_e[35] = 10000000000;
    uh_e[36] = 10000000000;
    uh_e[37] = 10000000000;
    uh_e[38] = 10000000000;
    uh_e[39] = 10000000000;
    uh_e[40] = 10000000000;
    uh_e[41] = 10000000000;
    uh_e[42] = 10000000000;
    uh_e[43] = 10000000000;
    uh_e[44] = 10000000000;
    uh_e[45] = 10000000000;
    uh_e[46] = 10000000000;
    uh_e[47] = 10000000000;
    uh_e[48] = 10000000000;
    uh_e[49] = 10000000000;
    uh_e[50] = 10000000000;
    uh_e[51] = 10000000000;
    uh_e[52] = 10000000000;
    uh_e[53] = 10000000000;
    uh_e[54] = 10000000000;
    uh_e[55] = 10000000000;
    uh_e[56] = 10000000000;
    uh_e[57] = 10000000000;
    uh_e[58] = 10000000000;
    uh_e[59] = 10000000000;
    uh_e[60] = 10000000000;
    uh_e[61] = 10000000000;
    uh_e[62] = 10000000000;
    uh_e[63] = 10000000000;
    uh_e[64] = 10000000000;
    uh_e[65] = 10000000000;
    uh_e[66] = 10000000000;
    uh_e[67] = 10000000000;
    uh_e[68] = 10000000000;
    uh_e[69] = 10000000000;
    uh_e[70] = 10000000000;
    uh_e[71] = 10000000000;
    uh_e[72] = 10000000000;
    uh_e[73] = 10000000000;
    uh_e[74] = 10000000000;
    uh_e[75] = 10000000000;
    uh_e[76] = 10000000000;
    uh_e[77] = 10000000000;
    uh_e[78] = 10000000000;
    uh_e[79] = 10000000000;
    uh_e[80] = 10000000000;
    uh_e[81] = 10000000000;
    uh_e[82] = 10000000000;
    uh_e[83] = 10000000000;
    uh_e[84] = 10000000000;
    uh_e[85] = 10000000000;
    uh_e[86] = 10000000000;
    uh_e[87] = 10000000000;
    uh_e[88] = 10000000000;
    uh_e[89] = 10000000000;
    uh_e[90] = 10000000000;
    uh_e[91] = 10000000000;
    uh_e[92] = 10000000000;
    uh_e[93] = 10000000000;
    uh_e[94] = 10000000000;
    uh_e[95] = 10000000000;
    uh_e[96] = 10000000000;
    uh_e[97] = 10000000000;
    uh_e[98] = 10000000000;
    uh_e[99] = 10000000000;
    uh_e[100] = 10000000000;
    uh_e[101] = 10000000000;
    uh_e[102] = 10000000000;
    uh_e[103] = 10000000000;
    uh_e[104] = 10000000000;
    uh_e[105] = 10000000000;
    uh_e[106] = 10000000000;
    uh_e[107] = 10000000000;
    uh_e[108] = 10000000000;
    uh_e[109] = 10000000000;
    uh_e[110] = 10000000000;
    uh_e[111] = 10000000000;
    uh_e[112] = 10000000000;
    uh_e[113] = 10000000000;
    uh_e[114] = 10000000000;
    uh_e[115] = 10000000000;
    uh_e[116] = 10000000000;
    uh_e[117] = 10000000000;
    uh_e[118] = 10000000000;
    uh_e[119] = 10000000000;
    uh_e[120] = 10000000000;
    uh_e[121] = 10000000000;
    uh_e[122] = 10000000000;
    uh_e[123] = 10000000000;
    uh_e[124] = 10000000000;
    uh_e[125] = 10000000000;
    uh_e[126] = 10000000000;
    uh_e[127] = 10000000000;
    uh_e[128] = 10000000000;
    uh_e[129] = 10000000000;
    uh_e[130] = 10000000000;
    uh_e[131] = 10000000000;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N, "uh", uh_e);
    free(luh_e);















}

// this function only sets external functions, numerical values are set in omni_mpc_acados_create_setup_nlp_in_numerical_values
void omni_mpc_acados_create_setup_nlp_in(omni_mpc_solver_capsule* capsule, const int N)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
        
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac_hess",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    
    
    
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
        
        
        
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);
    
    
    

    /**** Constraints ****/

    // bounds for initial stage

    // set up nonlinear constraints for initial stage
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun_jac", &capsule->nl_constr_h_0_fun_jac);
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun", &capsule->nl_constr_h_0_fun);
    
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun_jac_hess",
                                  &capsule->nl_constr_h_0_fun_jac_hess);
    
    
    
    


    // set up nonlinear constraints for stage 1 to N-1
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i,
                                      "nl_constr_h_fun_jac_hess", &capsule->nl_constr_h_fun_jac_hess[i-1]);
        
        
      
        
    }



    /* terminal constraints */

    // set up nonlinear constraints for last stage

    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac", &capsule->nl_constr_h_e_fun_jac);
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun", &capsule->nl_constr_h_e_fun);
    
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac_hess",
                                  &capsule->nl_constr_h_e_fun_jac_hess);
    
    
  
    }


static void omni_mpc_acados_create_set_opts(omni_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);

    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    double globalization_alpha_min = 0.05;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_min", &globalization_alpha_min);

    double globalization_alpha_reduction = 0.7;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_reduction", &globalization_alpha_reduction);



    int globalization_line_search_use_sufficient_descent = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_line_search_use_sufficient_descent", &globalization_line_search_use_sufficient_descent);

    int globalization_use_SOC = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_use_SOC", &globalization_use_SOC);

    double globalization_eps_sufficient_descent = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_eps_sufficient_descent", &globalization_eps_sufficient_descent);

    int with_solution_sens_wrt_params_forw = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_forw", &with_solution_sens_wrt_params_forw);

    int with_solution_sens_wrt_params_adj = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_adj", &with_solution_sens_wrt_params_adj);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    double levenberg_marquardt = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 30;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);
    double reg_epsilon = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_epsilon", &reg_epsilon);
    double reg_max_cond_block = 10000000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_max_cond_block", &reg_max_cond_block);

    double reg_min_epsilon = 0.00000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_min_epsilon", &reg_min_epsilon);

    bool reg_adaptive_eps = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_adaptive_eps", &reg_adaptive_eps);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 20;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    // NLP QP tol strategy
    ocp_nlp_qp_tol_strategy_t nlp_qp_tol_strategy = FIXED_QP_TOL;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_strategy", &nlp_qp_tol_strategy);

    double nlp_qp_tol_reduction_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_reduction_factor", &nlp_qp_tol_reduction_factor);

    double nlp_qp_tol_safety_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_safety_factor", &nlp_qp_tol_safety_factor);

    double nlp_qp_tol_min_stat = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_stat", &nlp_qp_tol_min_stat);

    double nlp_qp_tol_min_eq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_eq", &nlp_qp_tol_min_eq);

    double nlp_qp_tol_min_ineq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_ineq", &nlp_qp_tol_min_ineq);

    double nlp_qp_tol_min_comp = 0.00000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_comp", &nlp_qp_tol_min_comp);
    double timeout_max_time = 0.025;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "timeout_max_time", &timeout_max_time);

    ocp_nlp_timeout_heuristic_t timeout_heuristic = LAST;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "timeout_heuristic", &timeout_heuristic);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 60;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    double qp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_stat", &qp_solver_tol_stat);
    double qp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_eq", &qp_solver_tol_eq);
    double qp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_ineq", &qp_solver_tol_ineq);
    double qp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_comp", &qp_solver_tol_comp);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}


/**
 * Internal function for omni_mpc_acados_create: step 7
 */
void omni_mpc_acados_set_nlp_out(omni_mpc_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for omni_mpc_acados_create: step 9
 */
int omni_mpc_acados_create_precompute(omni_mpc_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int omni_mpc_acados_create_with_discretization(omni_mpc_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != OMNI_MPC_N && !new_time_steps) {
        fprintf(stderr, "omni_mpc_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, OMNI_MPC_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    omni_mpc_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = omni_mpc_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    omni_mpc_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    omni_mpc_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    omni_mpc_acados_create_setup_functions(capsule);
    omni_mpc_acados_create_setup_nlp_in(capsule, N);
    omni_mpc_acados_create_setup_nlp_in_numerical_values(capsule, N, new_time_steps);
    omni_mpc_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = omni_mpc_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int omni_mpc_acados_update_qp_solver_cond_N(omni_mpc_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from omni_mpc_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = omni_mpc_acados_create_precompute(capsule);
    return status;
}


int omni_mpc_acados_reset(omni_mpc_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar)
{
    // set initialization to all zeros
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    // sets primal and dual iterates to zero
    ocp_nlp_out_set_values_to_zero(nlp_config, nlp_dims, nlp_out);

    // reset integrator memory
    ocp_nlp_solver_reset_integrator_memory(nlp_solver, nlp_in, nlp_out);
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    if (reset_numerical_values)
    {
        // reset parameters to initial values
        omni_mpc_acados_create_set_default_parameters(capsule);

        // reset numerical values in nlp_in
        omni_mpc_acados_create_setup_nlp_in_numerical_values(capsule, N, NULL);
    }

    if (reset_solver_options)
    {
        // reset solver options to initial values
        omni_mpc_acados_create_set_opts(capsule);
    }

    if (reset_x_to_x0_bar)
    {double* buffer = calloc(NX, sizeof(double));
        ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
        for (int i=0; i<N+1; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        }
        free(buffer);
    }
    return 0;
}




int omni_mpc_acados_update_params(omni_mpc_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 540;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int omni_mpc_acados_update_params_sparse(omni_mpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int omni_mpc_acados_set_p_global_and_precompute_dependencies(omni_mpc_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, omni_mpc_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int omni_mpc_acados_solve(omni_mpc_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int omni_mpc_acados_setup_qp_matrices_and_factorize(omni_mpc_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int omni_mpc_acados_free(omni_mpc_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        

        
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt_hess[i]);
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
  
  
  
    free(capsule->discr_dyn_phi_fun_jac_ut_xt_hess);

    // cost
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    
    
    
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
        
        
        
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);
    
    
    

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac_hess[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    free(capsule->nl_constr_h_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun_jac_hess);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun_jac);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_e_fun_jac_hess);



    return 0;
}


void omni_mpc_acados_print_stats(omni_mpc_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[336];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int omni_mpc_acados_custom_update(omni_mpc_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *omni_mpc_acados_get_nlp_in(omni_mpc_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *omni_mpc_acados_get_nlp_out(omni_mpc_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *omni_mpc_acados_get_sens_out(omni_mpc_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *omni_mpc_acados_get_nlp_solver(omni_mpc_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *omni_mpc_acados_get_nlp_config(omni_mpc_solver_capsule* capsule) { return capsule->nlp_config; }
void *omni_mpc_acados_get_nlp_opts(omni_mpc_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *omni_mpc_acados_get_nlp_dims(omni_mpc_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *omni_mpc_acados_get_nlp_plan(omni_mpc_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
