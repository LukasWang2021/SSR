/**g++ -o filesManagerApi filesManagerApi.cpp -I/usr/include/python2.7 -L/usr/lib64/python2.7/config -lpython2.7 | ./filesManagerApi **/

#include "../src/serverAlarmApi.h"
#include <stdio.h>
#include <time.h>
#include <map>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <list>

using namespace std;
using std::string;
using std::list;
#define err_print(err_info) printf("\033[31;40m%s\033[0m\r\n", err_info);

ServerAlarmApi *p = ServerAlarmApi::GetInstance();
int main(int argc, char** argv)
{
    int statusCode;
    printf("start main....\r\n");

    int loopCnt = 100000;
    // 0x0001000100A70000 ["TpComm log{0}","TpComm模块日志{0}"]
    long long logCode = 0x0001000100A70000; //4295163914;
    std::cout<<"000000000";
    p->sendOneAlarm(logCode, "hello23");

    // "0001000900B50003":	["no expression present","找不到表达式"]	,
    // p->sendOneAlarm(0x0001000900B50003);

    logCode = 0x10003000A;

    while(loopCnt--){
        p->sendOneAlarm(logCode, "hello");   // 10003000A
        logCode += 1;
        printf("%d:发送报警:%016llX\r\n", loopCnt, logCode);

//        sleep(1);
//        pResult = p->sendOneAlarm(logCode);   // 10003000A
//        logCode += 1;
//        cout << loopCnt << ":发送报警:"<<logCode<<",响应值" << pResult << endl;
        usleep(100000/10); // 100ms
    }
    sleep(2);
    std::cout << "main  exit" << endl;
    return 0;
}

