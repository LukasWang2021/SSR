/*************************************************************************
	> File Name: log_manager_fst_logger.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时01分09秒
 ************************************************************************/

#include <iostream>
#include "log_manager/log_manager_transmitter.h"

int main(int argc, char **argv)
{
    fst_log::LogTransmitter logger;
    for (int i = 0; i < 10; ++i)
        logger.logInfo("hello world: %d\n", i);

    std::cout << "test end " << std::endl;

    return 0;
}
