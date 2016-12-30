/*************************************************************************
	> File Name: log_manager_fst_logger.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时01分09秒
 ************************************************************************/

#include <unistd.h>
#include <iostream>
#include "log_manager/log_manager_logger.h"

#include <comm_interface/comm_interface.h>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    fst_log::Logger log;
    if (!log.initLogger("transmitter1")) {
        return 0;
    }
    usleep(200 * 1000);
    fst_log::Logger log2;
    if (!log2.initLogger("transmitter2")) {
        cout << "err " << endl;
        return 0;
    }
    usleep(200 * 1000);

    log.info("hello world");
    log.warn("warnning: begin loop");
    for (int cnt = 0; cnt < 50; ++cnt) {
        log.info("This is the %d line", cnt);
    }
    log.error("end");

    std::string tmp = "sdfjiosafpasfd";
    log2.info(tmp);
    tmp = "gj;kf;jkgh;j";
    log2.warn(tmp);
    tmp = "324354256";
    log2.error(tmp);

    sleep(1);

 
    return 0;
}
