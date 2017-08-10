#include "sub_functions.h"
#include "common.h"


void printDbLine(const char* info, double* params, int len)
{
    FST_PRINT("%s:", info);
    for (int i = 0; i < len; i++)
    {
        FST_PRINT("%f ", params[i]);
    }
    FST_PRINT("\n");	
}
