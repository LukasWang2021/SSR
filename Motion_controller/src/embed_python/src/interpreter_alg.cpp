#include "interpreter_alg.h"
#include "basic_alg.h"
#include "log_manager_producer.h"

using namespace basic_alg;
using namespace log_space;

ErrorCode InterpLinealg_Inverse(const double *p_matrix, int dim, double *p_inv)
{
    int32_t ret = inverse(p_matrix, dim, p_inv);

    if(ret == 0) 
    {
        return SUCCESS;
    }
    else
    {
        LogProducer::error("InterpLinealg", "calculate matrix inverse failed return %d", ret);
        return INTERPRETER_ERROR_MOD_CALC_FAILED;
    }
}

ErrorCode InterpLinealg_Eigens(const double *p_matrix, int dim, double *eig_vec, double *eig_val)
{
    int32_t ret = eigens(p_matrix, dim, eig_vec, eig_val);
    if(ret == 0) 
    {
        return SUCCESS;
    }
    else
    {
        LogProducer::error("InterpLinealg", "calculate matrix eigens failed return %d", ret);
        return INTERPRETER_ERROR_MOD_CALC_FAILED;
    }
}
