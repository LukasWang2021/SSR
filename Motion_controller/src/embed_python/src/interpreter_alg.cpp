#include "interpreter_alg.h"
#include "basic_alg.h"

using namespace basic_alg;

ErrorCode InterpLinealg_Inverse(const double *p_matrix, int dim, double *p_inv)
{
    if(inverse(p_matrix, dim, p_inv)) return SUCCESS;

    return -1;
}

ErrorCode InterpLinealg_Eigens(const double *p_matrix, int dim, double *eig_vec, double *eig_val)
{
    if(eigens(p_matrix, dim, eig_vec, eig_val)) return 0;

    return -1;
}
