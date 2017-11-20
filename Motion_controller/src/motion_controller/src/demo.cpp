#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>


#include <log_manager/log_manager_logger.h>
#include <motion_controller/motion_controller_lockfree_fifo.h>



#define PI          3.1415926
#define MAP_SIZE    1000000
double sin_map[MAP_SIZE];

double sin_maq[MAP_SIZE];

double x[10000];
double yy1[10000];
double yy2[10000];
double yy3[10000];

fst_log::Logger log1;

void buildMap()
{
    clock_t start, finish;
    
    start = clock();
    for (int i = 0; i < MAP_SIZE; i++) {
        sin_map[i] = sin(PI * i / MAP_SIZE);
        sin_maq[i] = sin(i / MAP_SIZE);
    }
    finish = clock();

    log1.info("time consumed by map builder: %fs", (double)(finish - start) / CLOCKS_PER_SEC);
    
    for (int i = 0; i < 10000; i++) {
        x[i] =  i / 10000;
    }
}


void testSine()
{
    clock_t start, finish;
    double tmp;
    double res = 0;

    start = clock();
    for (int lo = 0; lo < 10000; lo++) {
        for (int i = 0; i < 10000; i++)
            yy1[i] = sin(x[i]);
    }
    finish = clock();

    for (int i = 0; i < 10000; i++) {
        res += yy1[i];
    }

    log1.info("res=%f, time consumed by sin: %fs", res, (double)(finish - start) / CLOCKS_PER_SEC);
}

void testMap()
{
    clock_t start, finish;
    double tmp;
    double res = 0;

    start = clock();
    for (int lo = 0; lo < 10000; lo++) {
        for (int i = 0; i < 10000; i++)
            yy2[i] = sin_map[(unsigned int)(x[i] * 318309.85108510)]; 
    }
    finish = clock();

    for (int i = 0; i < 10000; i++) {
        res += yy2[i];
    }

    log1.info("res = %f, time consumed by map: %fs", res, (double)(finish - start) / CLOCKS_PER_SEC);
}

void testMaq()
{
    clock_t start, finish;
    double tmp;
    double res = 0;

    start = clock();
    for (int lo = 0; lo < 10000; lo++) {
        for (int i = 0; i < 10000; i++)
            yy3[i] = sin_maq[(unsigned int)(x[i] * MAP_SIZE)]; 
    }
    finish = clock();

    for (int i = 0; i < 10000; i++) {
        res += yy3[i];
    }

    log1.info("res = %f, time consumed by map: %fs", res, (double)(finish - start) / CLOCKS_PER_SEC);
}

void testLockFreeFIFO(void)
{
    fst_controller::LockFreeFIFO<double> *fifo = new fst_controller::LockFreeFIFO<double>(500);
    fifo->pushItem(1.0);
    fifo->pushItem(1.1);
    fifo->pushItem(1.2);
    fifo->pushItem(1.3);

    std::cout << "fifo size = " << fifo->size() << std::endl;
    double a, b, c, d;
    fifo->fetchItem(a);
    fifo->fetchItem(b);
    std::cout << "fifo size = " << fifo->size() << std::endl;
    fifo->fetchItem(c);
    fifo->fetchItem(d);
    std::cout << "fifo size = " << fifo->size() << std::endl;
    std::cout << a << ", " << b << ", " << c << ", " << d << std::endl;

    fst_controller::LockFreeFIFO<double> fifo1(100);
    fifo1.pushItem(5.5);
    std::cout << "fifo size = " << fifo1.size() << std::endl;
    fifo1.fetchItem(c);
    std::cout << c << std::endl;
    std::cout << "fifo size = " << fifo1.size() << std::endl;

}

void ala()
{
    double sum1 = 0;
    double sum2 = 0;
    double tmp;
    for (int i = 0; i < 10000; i++) {
        tmp = yy1[i] - yy2[i];
        sum1 += tmp * tmp;
        tmp = yy1[i] - yy3[i];
        sum2 += tmp * tmp;
    }

    sum1 = sqrt(sum1);
    sum2 = sqrt(sum2);
    log1.info("sum1=%f, sum2=%f", sum1, sum2);
}

int main(int argc, char **argv)
{
    log1.initLogger("sin_map");
    log1.info("hello world");
    
	int loop = 0;
    int i = 0;
    struct timeval time_now, time_past;

    //buildMap();

    //testSine();
    //testMap();
    //testMaq();

    //ala();
    //
    testLockFreeFIFO();

    return 0;
}


