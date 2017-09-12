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
#include "log_manager/log_manager_logger.h"
#include <stdarg.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <stdexcept>
#include <log_manager_version.h>

#define random(x) (rand()%x)

using std::cout;
using std::endl;

bool g_running = true;
fst_log::Logger log_0_1;
fst_log::Logger log_2_6;
fst_log::Logger log_7;
fst_log::Logger log_8;
fst_log::Logger log_9;


void thread0(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_0_1.info("Thread0 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_0_1.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_0_1.warn("Thread0 is terminated");
}

void thread1(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_0_1.info("Thread1 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_0_1.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_0_1.warn("Thread1 is terminated");
}

void thread2(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.info("Thread2 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_2_6.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread2 is terminated");
}

void thread3(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.info("Thread3 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_2_6.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread3 is terminated");
}

void thread4(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.info("Thread4 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_2_6.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread4 is terminated");
}

void thread5(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.info("Thread5 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_2_6.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread5 is terminated");
}

void thread6(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_2_6.info("Thread6 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_2_6.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_2_6.warn("Thread6 is terminated");
}

void thread7(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_7.info("Thread7 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_7.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_7.warn("Thread7 is terminated");
}

void thread8(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_8.info("Thread8 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_8.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_8.warn("Thread8 is terminated");
}

void thread9(void)
{
    int loop = 0;
    int sleep;
    while (g_running) {
        log_9.info("Thread9 is running loop cnt=%d", loop);
        loop++;
        sleep = random(100);
        log_9.info("  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log_9.warn("Thread9 is terminated");
}

static void sigintHandle(int num)
{
    g_running = false;
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandle);
    printf("Start log-manager TEST program.\n");

    log_0_1.initLogger("log_0_1");
    log_2_6.initLogger("log_2_6");
    log_7.initLogger("log_7");
    log_8.initLogger("log_8");
    log_9.initLogger("log_9");

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

    printf("Test end.\n");
    return 0;
}
