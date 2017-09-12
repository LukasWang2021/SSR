#include "sub_functions.h"
#include "common.h"
//nclude <thread> 


void printDbLine(const char* info, double* params, int len)
{
    FST_PRINT("%s:", info);
    for (int i = 0; i < len; i++)
    {
        FST_PRINT("%f ", params[i]);
    }
    FST_PRINT("\n");	
}

int timeval_subtract(struct timeval* result, struct timeval* x, struct timeval* y)   
{   
    if ( x->tv_sec>y->tv_sec )   
              return -1;   
  
    if ( (x->tv_sec==y->tv_sec) && (x->tv_usec>y->tv_usec) )   
              return -1;   
  
    result->tv_sec = ( y->tv_sec-x->tv_sec );   
    result->tv_usec = ( y->tv_usec-x->tv_usec );   
  
    if (result->tv_usec<0)   
    {   
              result->tv_sec--;   
              result->tv_usec+=1000000;   
    }   
  
    return 0;   
}
