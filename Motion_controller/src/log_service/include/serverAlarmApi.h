#ifndef ServerAlarmApi_2018_6_22
#define ServerAlarmApi_2018_6_22

#include <Python.h>
#include <string>
#include <list>
using  std::list;
using  std::string;

class ServerAlarmApi
{
private:
    ServerAlarmApi();
    char m_buf[32];
    bool enabled;
    static ServerAlarmApi *m_pInstance;
public:
    bool m_loop;
    pthread_t m_tid;
    pthread_cond_t m_cond;
    pthread_mutex_t m_lock;
    list<string> m_record_list;
    static ServerAlarmApi *GetInstance();
    int sendOneAlarm(unsigned long long log_code, string param);
    int sendOneAlarm(unsigned long long log_code);
    void pyDecref();
    void setEnable(bool enable_status);
};
#endif

