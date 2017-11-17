/*************************************************************************
	> File Name: log_manager_fst_logger.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时01分09秒
 ************************************************************************/

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <stdexcept>
#include <sys/time.h>
#include <sys/mman.h>  
#include <fcntl.h>  
#include <sys/stat.h> 
#include <log_manager_version.h>
#include "log_manager/log_manager_logger.h"

#define random(x) (rand()%x)

using std::cout;
using std::endl;

using fst_log::ControlArea;
using fst_log::LogItem;

bool g_running = true;
fst_log::Logger log_0_1;
fst_log::Logger log_2_6;
fst_log::Logger log_7;
fst_log::Logger log_8;
fst_log::Logger log_9;

fst_log::Logger log_10;


void thread0(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_0_1.log("Thread0 is running loop cnt=%d", loop);
        loop++;
        sleep = random(10);
        log_0_1.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_0_1.warn("Thread0 is terminated");
}

void thread1(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_0_1.log("Thread1 is running loop cnt=%d", loop);
        loop++;
        sleep = random(10);
        log_0_1.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_0_1.warn("Thread1 is terminated");
}

void thread2(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.log("Thread2 is running loop cnt=%d", loop);
        loop++;
        sleep = random(10);
        log_2_6.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread2 is terminated");
}

void thread3(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.log("Thread3 is running loop cnt=%d", loop);
        loop++;
        sleep = random(50);
        log_2_6.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread3 is terminated");
}

void thread4(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.log("Thread4 is running loop cnt=%d", loop);
        loop++;
        sleep = random(10);
        log_2_6.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread4 is terminated");
}

void thread5(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.log("Thread5 is running loop cnt=%d", loop);
        loop++;
        sleep = random(5);
        log_2_6.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread5 is terminated");
}

void thread6(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.log("Thread6 is running loop cnt=%d", loop);
        loop++;
        sleep = random(10);
        log_2_6.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread6 is terminated");
}

void thread7(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_7.log("Thread7 is running loop cnt=%d", loop);
        loop++;
        sleep = random(3);
        log_7.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_7.warn("Thread7 is terminated");
}

void thread8(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_8.log("Thread8 is running loop cnt=%d", loop);
        loop++;
        sleep = random(5);
        log_8.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_8.warn("Thread8 is terminated");
}

void thread9(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_9.log("Thread9 is running loop cnt=%d", loop);
        loop++;

        /*
        if (loop == 50) {
            log_9.warn("do something bad");
            int *p = new int[10];
            memset(p - 200, 178, 1000);
            delete[] p;
            log_9.error("done");
        }
        */
        
        sleep = random(6);
        log_9.log("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_9.warn("Thread9 is terminated");
}

void thread10(void)
{
    int loop = 0;
    int rept = 25;
    int sleep = 1;

    while (g_running) {
        for (int tmp = 0; tmp < rept; ++tmp)
            log_10.log("%dk items per sec: %d", rept * 10, tmp);

        usleep(sleep * 100);
    }

    log_9.warn("Thread10 is terminated");
}


static void sigintHandle(int num)
{
    g_running = false;
}

void test(void);

int main(int argc, char **argv)
{

    signal(SIGINT, sigintHandle);
    printf("Start log-manager TEST program.\n");

    struct timeval time_now;
    printf("here\n");

    log_0_1.initLogger("log_0_1");
    printf("init log_0_1 done\n");
    log_2_6.initLogger("log_2_6");
    printf("init log_2_6 done\n");
    log_7.initLogger("log_7");
    printf("init log_7 done\n");
    log_8.initLogger("log_8");
    printf("init log_8 done\n");
    log_9.initLogger("log_9");
    printf("init log_9 done\n");
    log_10.initLogger("log10");
    printf("init log_10 done\n");

    /*
    boost::thread t0(&thread0);
    boost::thread t1(&thread1);
    boost::thread t2(&thread2);
    boost::thread t3(&thread3);
    boost::thread t4(&thread4);
    boost::thread t5(&thread5);
    boost::thread t6(&thread6);
    boost::thread t7(&thread7);
    boost::thread t8(&thread8);
    boost::thread t9(&thread9);
    */
    boost::thread t10(&thread10);

    /*
    t0.join();
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();
    t8.join();
    t9.join();
    */
    t10.join();

    printf("Test end.\n");
    return 0;
}
