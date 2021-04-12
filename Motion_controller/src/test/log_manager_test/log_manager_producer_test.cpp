#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <map>
#include <stdexcept>
#include <sys/time.h>
#include <sys/mman.h>  
#include <fcntl.h>  
#include <sys/stat.h> 

#include "thread_help.h"
#include "log_manager_producer.h"
#include "log_manager_producer_bare.h"

using namespace log_space;

#define random(x) (rand()%x)

bool g_running = true;
log_space::LogProducer g_share_log;


void* thread0(void*)
{
    printf("init log_0 start\n");
    uint32_t isr_count = 1;
	uint32_t fd = initLogProducerBare("barecore0", &isr_count);
    printf("init log_0 done, fd = %u\n", fd);

    int loop = 0;
    int sleep;
    while (g_running) {
        infoBare(fd, "Thread0 is running loop cnt");
        loop++;
        sleep = random(10);
        warnBare(fd,"Thread0 sleep ms");
        usleep(sleep * 1000);
    }
    warnBare(fd,"Thread0 is terminated");
	unblindLogProducerBare(fd);
	return NULL;
}

void* thread1(void*)
{
    uint32_t isr_count = 2;
	uint32_t fd = initLogProducerBare("barecore1", &isr_count);
    printf("init log_1 done, fd = %u\n", fd);

    int loop = 0;
    int sleep;
    while (g_running) {
        infoBare(fd, "Thread1 is running loop cnt");
        loop++;
        sleep = random(10);
        warnBare(fd,"Thread1 sleep ms");
        usleep(sleep * 1000);
    }
    warnBare(fd,"Thread1 is terminated");
	unblindLogProducerBare(fd);
	return NULL;
}

void* thread2(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log2", &isr_count);
    printf("init log_2 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module2","Thread2 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module2","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.warn("module2","Thread2 is terminated");
	return NULL;
}

void* thread3(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log3", &isr_count);
    printf("init log_3 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module3","Thread3 is running loop cnt=%d", loop);
        loop++;
        sleep = random(50);
        LogProducer::warn("module3","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.warn("module3","Thread3 is terminated");
	return NULL;
}

void* thread4(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log4", &isr_count);
    printf("init log_4 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module4","Thread4 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module4","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.warn("module4","Thread4 is terminated");
	return NULL;
}

void* thread5(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log5", &isr_count);
    printf("init log_5 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module5","Thread5 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module5","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.warn("module5","Thread5 is terminated");
	return NULL;
}

void* thread6(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log6", &isr_count);
    printf("init log_6 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module6","Thread6 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module6","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.error("module6","Thread6 is terminated");
	return NULL;
}

void* thread7(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log7", &isr_count);
    printf("init log_7 done\n");

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module7","Thread7 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module7","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.error("module7","Thread7 is terminated");
	return NULL;
}

void* thread8(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log8", &isr_count);
    pthread_t pid = pthread_self();
    printf("init log_8 done, id = %lu\n", pid);

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module8","Thread8 is running loop cnt=%d", loop);
        loop++;
        sleep = random(20);
        LogProducer::warn("module8","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.error("module8","Thread8 is terminated");
	return NULL;
}

void* thread9(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log9", &isr_count);
    pthread_t pid = pthread_self();
    printf("init log_9 done, id = %lu\n", pid);

    int loop = 0;
    int sleep;
    while (g_running) {
        LogProducer::info("module9","Thread9 is running loop cnt=%d", loop);
        loop++;
        
        sleep = random(20);
        LogProducer::warn("module9","  sleep %d ms", sleep);
        usleep(sleep * 1000);
    }
    log.error("module9","Thread9 is terminated");
	return NULL;
}

void* thread10(void*)
{
    uint32_t isr_count = 1;
	log_space::LogProducer log;
	log.init("log10", &isr_count);
    pthread_t pid = pthread_self();
    printf("init log_10 done, id = %lu\n", pid);

    int rept = 1000;
    int sleep = 10;

    while (g_running) {
        for (int tmp = 0; tmp < rept; ++tmp)
        {
            LogProducer::info("module10","%d items per sec: %d", rept, tmp);
            LogProducer::warn("module10","  sleep %d ms", sleep);
			usleep(sleep * 1000);
        }
    }

    printf("Thread10 is terminated\n");
	return NULL;
}


static void sigintHandle(int num)
{
    g_running = false;
}


int main(int argc, char **argv)
{

    signal(SIGINT, sigintHandle);
	signal(SIGTERM, sigintHandle);
    printf("Start log-manager_producer TEST program.\n");


    uint32_t isr_count = 10;
	g_share_log.init("main", &isr_count);
    LogProducer::info("none","log in main init");

    //base_space::ThreadHelp t0; used in barecore
	//base_space::ThreadHelp t1; used in barecore
	base_space::ThreadHelp t2;
	base_space::ThreadHelp t3;
	base_space::ThreadHelp t4;
	base_space::ThreadHelp t5;
	base_space::ThreadHelp t6;
	base_space::ThreadHelp t7;
	base_space::ThreadHelp t8;
	base_space::ThreadHelp t9;
	base_space::ThreadHelp t10;
	
	//t0.run(&thread0, NULL, 20);
	//t1.run(&thread1, NULL, 20);
	t2.run(&thread2, NULL, 20);
	t3.run(&thread3, NULL, 20);
	t4.run(&thread4, NULL, 20);
	t5.run(&thread5, NULL, 20);
	t6.run(&thread6, NULL, 20);
	t7.run(&thread7, NULL, 20);
	t8.run(&thread8, NULL, 20);
	t9.run(&thread9, NULL, 20);
	t10.run(&thread10, NULL, 20);


    
    //t0.join();
    //t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();
    t8.join();
    t9.join();
    t10.join();

    printf("Test end.\n");
    return 0;
}
