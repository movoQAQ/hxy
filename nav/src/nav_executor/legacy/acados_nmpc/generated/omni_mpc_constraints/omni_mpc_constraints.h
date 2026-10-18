/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef omni_mpc_CONSTRAINTS
#define omni_mpc_CONSTRAINTS

#ifdef __cplusplus
extern "C" {
#endif









int omni_mpc_constr_h_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_fun_jac_uxt_zt_sparsity_in(int);
const int *omni_mpc_constr_h_fun_jac_uxt_zt_sparsity_out(int);
int omni_mpc_constr_h_fun_jac_uxt_zt_n_in(void);
int omni_mpc_constr_h_fun_jac_uxt_zt_n_out(void);

int omni_mpc_constr_h_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_fun_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_fun_sparsity_in(int);
const int *omni_mpc_constr_h_fun_sparsity_out(int);
int omni_mpc_constr_h_fun_n_in(void);
int omni_mpc_constr_h_fun_n_out(void);







int omni_mpc_constr_h_fun_jac_uxt_zt_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_fun_jac_uxt_zt_hess_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_fun_jac_uxt_zt_hess_sparsity_in(int);
const int *omni_mpc_constr_h_fun_jac_uxt_zt_hess_sparsity_out(int);
int omni_mpc_constr_h_fun_jac_uxt_zt_hess_n_in(void);
int omni_mpc_constr_h_fun_jac_uxt_zt_hess_n_out(void);




int omni_mpc_constr_h_0_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_0_fun_jac_uxt_zt_sparsity_in(int);
const int *omni_mpc_constr_h_0_fun_jac_uxt_zt_sparsity_out(int);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_n_in(void);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_n_out(void);

int omni_mpc_constr_h_0_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_0_fun_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_0_fun_sparsity_in(int);
const int *omni_mpc_constr_h_0_fun_sparsity_out(int);
int omni_mpc_constr_h_0_fun_n_in(void);
int omni_mpc_constr_h_0_fun_n_out(void);








int omni_mpc_constr_h_0_fun_jac_uxt_zt_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_hess_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_0_fun_jac_uxt_zt_hess_sparsity_in(int);
const int *omni_mpc_constr_h_0_fun_jac_uxt_zt_hess_sparsity_out(int);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_hess_n_in(void);
int omni_mpc_constr_h_0_fun_jac_uxt_zt_hess_n_out(void);





int omni_mpc_constr_h_e_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_e_fun_jac_uxt_zt_sparsity_in(int);
const int *omni_mpc_constr_h_e_fun_jac_uxt_zt_sparsity_out(int);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_n_in(void);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_n_out(void);

int omni_mpc_constr_h_e_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_e_fun_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_e_fun_sparsity_in(int);
const int *omni_mpc_constr_h_e_fun_sparsity_out(int);
int omni_mpc_constr_h_e_fun_n_in(void);
int omni_mpc_constr_h_e_fun_n_out(void);








int omni_mpc_constr_h_e_fun_jac_uxt_zt_hess(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_hess_work(int *, int *, int *, int *);
const int *omni_mpc_constr_h_e_fun_jac_uxt_zt_hess_sparsity_in(int);
const int *omni_mpc_constr_h_e_fun_jac_uxt_zt_hess_sparsity_out(int);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_hess_n_in(void);
int omni_mpc_constr_h_e_fun_jac_uxt_zt_hess_n_out(void);



#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // omni_mpc_CONSTRAINTS
