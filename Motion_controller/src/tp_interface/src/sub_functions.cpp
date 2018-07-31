#include "sub_functions.h"
#include "common.h"
#include <fst_datatype.h>
#include <unistd.h>
#include <sys/time.h>


void printDbLine(const char* info, double* params, int len)
{
    FST_INFO("%s:", info);
    for (int i = 0; i < len; i++)
    {
        FST_INFO("%f ", params[i]);
    }
}

bool waitSignalTimeout(bool sig, int timeout)
{
    int interval = 100;
    int count = timeout / interval;
    while (!sig)
    {
        usleep (interval);
        if (count-- <= 0)
        {
            FST_ERROR("requeset timeout");
            return false;
        }
    }
    return true;
}


double get2PIDeltaValue(double value1, double value2)
{
    //FST_INFO("value1:%f, value2:%f", value1, value2);
    return round((value1 - value2) / (2 * PI)) * (2 * PI); 
}

long getCurTimeSecond()
{
    time_t seconds;

    seconds = time((time_t *)NULL);

    return seconds;
}

bool setTimeSecond(long seconds)
{
    struct timeval tv;
    tv.tv_sec = seconds;  
    tv.tv_usec = 0; 
    if(settimeofday (&tv, (struct timezone *) 0) < 0)  
    {  
        printf("Set system datatime error!/n");  
        return false;  
    }  
    return true;
}
