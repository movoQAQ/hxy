/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef omni_mpc_MODEL
#define omni_mpc_MODEL

#ifdef __cplusplus
extern "C" {
#endif




int omni_mpc_dyn_disc_phi_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_dyn_disc_phi_fun_work(int *, int *, int *, int *);
const int *omni_mpc_dyn_disc_phi_fun_sparsity_in(int);
const int *omni_mpc_dyn_disc_phi_fun_sparsity_out(int);
int omni_mpc_dyn_disc_phi_fun_n_in(void);
int omni_mpc_dyn_disc_phi_fun_n_out(void);

int omni_mpc_dyn_disc_phi_fun_jac(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_dyn_disc_phi_fun_jac_work(int *, int *, int *, int *);
const int *omni_mpc_dyn_disc_phi_fun_jac_sparsity_in(int);
const int *omni_mpc_dyn_disc_phi_fun_jac_sparsity_out(int);
int omni_mpc_dyn_disc_phi_fun_jac_n_in(void);
int omni_mpc_dyn_disc_phi_fun_jac_n_out(void);







int omni_mpc_dyn_disc_phi_fun_jac_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_dyn_disc_phi_fun_jac_hess_work(int *, int *, int *, int *);
const int *omni_mpc_dyn_disc_phi_fun_jac_hess_sparsity_in(int);
const int *omni_mpc_dyn_disc_phi_fun_jac_hess_sparsity_out(int);
int omni_mpc_dyn_disc_phi_fun_jac_hess_n_in(void);
int omni_mpc_dyn_disc_phi_fun_jac_hess_n_out(void);





#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // omni_mpc_MODEL
