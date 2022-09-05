#ifndef INTERPRETER_ALG_H
#define INTERPRETER_ALG_H

#include "common_error_code.h"

bool InterpLinealg_Init(void);

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode InterpLinealg_Inverse(const double *p_matrix, int dim, double *p_inv);
ErrorCode InterpLinealg_Eigens(const double *p_matrix, int dim, double *eig_vec, double *eig_val);

#ifdef __cplusplus
}
#endif

#endif
