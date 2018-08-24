#ifndef ServerAlarmApi_2018_6_22
#define ServerAlarmApi_2018_6_22

#include <Python.h>
#include <string>
#include <list>
using  std::string;
using  std::list;
class ServerAlarmApi
{
private:
    ServerAlarmApi();
    bool mInitFlag;
    char mBuf[32];
    PyObject *mPost;
    static ServerAlarmApi *m_pInstance;

public:
    static ServerAlarmApi *GetInstance();
    int sendOneAlarm(unsigned long long logCode, string param);
    int sendOneAlarm(unsigned long long logCode);
    void pyDecref();
};
#endif