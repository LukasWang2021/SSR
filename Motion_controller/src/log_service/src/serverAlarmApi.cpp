/**g++ -o files_manager_client27 files_manager_client27.cpp -I/usr/include/python2.7 -L/usr/lib64/python2.7/config -lpython2.7 | ./files_manager_client27 **/

#include "serverAlarmApi.h"
#include <time.h>
#include <iostream>
using namespace std;
using std::string;
ServerAlarmApi *ServerAlarmApi::m_pInstance = NULL;
void* threadFun(void *arg);
ServerAlarmApi::ServerAlarmApi():
    enabled(false)
{
    pthread_mutex_init(&m_lock, NULL);
    pthread_cond_init(&m_cond, NULL);
    m_loop = true;
    if(pthread_create(&m_tid,NULL,threadFun,this)!=0)
    {
        cout<<"create thread failed!"<<endl;
        return;
    }
    sleep(3);
}

ServerAlarmApi *ServerAlarmApi::GetInstance()
{
    if(m_pInstance == NULL)
        m_pInstance = new ServerAlarmApi();
    return m_pInstance;
}
void ServerAlarmApi::pyDecref()
{
    m_loop = false;
//    pthread_cancel(m_tid);
}
// @description 发送一条Log
// @param logCode: Log code
// @param param: 动态参数
// @return 返回执行结果, 返回状态码,1000表示执行成功,否则表示执行失败。失败时打印错误信息
int ServerAlarmApi::sendOneAlarm(unsigned long long log_code, string param)
{
    if(!enabled)
    {
        std::cout<<"LogService: 0x"<<std::hex<<log_code<<": "<<param<<std::endl;
        return 0;
    }
    pthread_mutex_lock(&m_lock);
    sprintf(m_buf, "%016llX", log_code);
    m_record_list.push_back(string(m_buf) + param);
    pthread_cond_signal(&m_cond);
    pthread_mutex_unlock(&m_lock);
    return 1000;
}
// @description 发送一条Log
// @param logCode: Log code
// @return 返回执行结果, 返回状态码,1000表示执行成功,否则表示执行失败。失败时打印错误信息
int ServerAlarmApi::sendOneAlarm(unsigned long long log_code)
{
    if(!enabled)
    {
        std::cout<<"LogService: 0x"<<std::hex<<log_code<<std::endl;
        return 0;
    }
    sprintf(m_buf, "%016llX", log_code);
    m_record_list.push_back(string(m_buf));
    return 1000;
}
// @description 线程函数,用于向报警服务发送报警
// @param *arg: 线程入口参数
// @return NULL
void* threadFun(void *arg)
{
    if ( !Py_IsInitialized() ) {
        Py_Initialize();
        PyEval_InitThreads();
    }
    if ( !Py_IsInitialized() ) {
        cout << "Py_Initialize failed " <<endl;
        return NULL;
    }
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/root/install/share/runtime/python/')");
    PyObject *pName = PyString_FromString("send_one_alarm");
    PyObject *pModule = PyImport_Import(pName);
    if (!pModule) {
        cout<<"Cant open python sendonealarm.py!"<<endl;
        return NULL;
    }
    PyObject *pDict = PyModule_GetDict(pModule);
    PyObject *postMethod = PyDict_GetItemString(pDict, "send_one_alarm");
    if (!postMethod) {
        cout<<"send_one_alarm is NULL!"<<endl;
        return NULL;
    }
    Py_XDECREF(pName);
    Py_XDECREF(pDict);
    Py_XDECREF(pModule);
    // 等待事件超时时间
    struct timeval now;
    struct timespec outtime;

    ServerAlarmApi *pServerApi = (ServerAlarmApi *)arg;
    list<string>::iterator iter;
    while(pServerApi->m_loop)
    {
        pthread_mutex_lock(&pServerApi->m_lock);
        gettimeofday(&now, NULL);
        outtime.tv_sec = now.tv_sec + 2; // 2second超时
        pthread_cond_timedwait(&pServerApi->m_cond, &pServerApi->m_lock, &outtime);
        for(iter = pServerApi->m_record_list.begin(); iter != pServerApi->m_record_list.end(); iter++)
        {
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();
            int statusCode;
            PyObject *pArgs = PyTuple_New(1);
            PyTuple_SetItem(pArgs, 0, Py_BuildValue("s",(*iter).c_str()));
            PyObject *result = PyObject_CallObject(postMethod, pArgs);
            Py_XDECREF(pArgs);
            if(result != NULL){
                PyArg_Parse(result, "i", &statusCode);// 解析失败返回0
                Py_XDECREF(result);
            }
            if(statusCode != 1000)
            {
                // 发送报警失败了,查看终端打印信息
            }
            PyGILState_Release(gstate);
        }
        pServerApi->m_record_list.clear();
        pthread_mutex_unlock(&pServerApi->m_lock);
    }
    cout<< "ServerAlarmApi exit thread" << endl;
    Py_DECREF(postMethod);
    Py_Finalize();
}

void ServerAlarmApi::setEnable(bool enable_status)
{
    enabled = enable_status;
}

