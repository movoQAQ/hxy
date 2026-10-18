/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */


#ifndef omni_mpc_COST
#define omni_mpc_COST

#ifdef __cplusplus
extern "C" {
#endif


// Cost at initial shooting node

int omni_mpc_cost_ext_cost_0_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_0_fun_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_0_fun_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_0_fun_sparsity_out(int);
int omni_mpc_cost_ext_cost_0_fun_n_in(void);
int omni_mpc_cost_ext_cost_0_fun_n_out(void);

int omni_mpc_cost_ext_cost_0_fun_jac_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_0_fun_jac_hess_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_0_fun_jac_hess_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_0_fun_jac_hess_sparsity_out(int);
int omni_mpc_cost_ext_cost_0_fun_jac_hess_n_in(void);
int omni_mpc_cost_ext_cost_0_fun_jac_hess_n_out(void);

int omni_mpc_cost_ext_cost_0_fun_jac(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_0_fun_jac_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_0_fun_jac_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_0_fun_jac_sparsity_out(int);
int omni_mpc_cost_ext_cost_0_fun_jac_n_in(void);
int omni_mpc_cost_ext_cost_0_fun_jac_n_out(void);









// Cost at path shooting node

int omni_mpc_cost_ext_cost_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_fun_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_fun_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_fun_sparsity_out(int);
int omni_mpc_cost_ext_cost_fun_n_in(void);
int omni_mpc_cost_ext_cost_fun_n_out(void);

int omni_mpc_cost_ext_cost_fun_jac_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_fun_jac_hess_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_fun_jac_hess_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_fun_jac_hess_sparsity_out(int);
int omni_mpc_cost_ext_cost_fun_jac_hess_n_in(void);
int omni_mpc_cost_ext_cost_fun_jac_hess_n_out(void);

int omni_mpc_cost_ext_cost_fun_jac(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_fun_jac_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_fun_jac_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_fun_jac_sparsity_out(int);
int omni_mpc_cost_ext_cost_fun_jac_n_in(void);
int omni_mpc_cost_ext_cost_fun_jac_n_out(void);








// Cost at terminal shooting node

int omni_mpc_cost_ext_cost_e_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_e_fun_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_e_fun_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_e_fun_sparsity_out(int);
int omni_mpc_cost_ext_cost_e_fun_n_in(void);
int omni_mpc_cost_ext_cost_e_fun_n_out(void);

int omni_mpc_cost_ext_cost_e_fun_jac_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_e_fun_jac_hess_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_e_fun_jac_hess_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_e_fun_jac_hess_sparsity_out(int);
int omni_mpc_cost_ext_cost_e_fun_jac_hess_n_in(void);
int omni_mpc_cost_ext_cost_e_fun_jac_hess_n_out(void);

int omni_mpc_cost_ext_cost_e_fun_jac(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_cost_ext_cost_e_fun_jac_work(int *, int *, int *, int *);
const int *omni_mpc_cost_ext_cost_e_fun_jac_sparsity_in(int);
const int *omni_mpc_cost_ext_cost_e_fun_jac_sparsity_out(int);
int omni_mpc_cost_ext_cost_e_fun_jac_n_in(void);
int omni_mpc_cost_ext_cost_e_fun_jac_n_out(void);









#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // omni_mpc_COST
