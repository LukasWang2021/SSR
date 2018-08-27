/**g++ -o files_manager_client27 files_manager_client27.cpp -I/usr/include/python2.7 -L/usr/lib64/python2.7/config -lpython2.7 | ./files_manager_client27 **/

#include "serverAlarmApi.h"
#include <iostream>
using namespace std;
using std::string;
ServerAlarmApi *ServerAlarmApi::m_pInstance = NULL;
ServerAlarmApi::ServerAlarmApi():
    enabled(false)
{
    mInitFlag = 0;
    if ( !Py_IsInitialized() ) {
        Py_Initialize();
    }
    if ( !Py_IsInitialized() ) {
        mInitFlag -= 1;
    }
    
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/root/install/share/runtime/python/')");
    PyObject *pName = PyString_FromString("send_one_alarm");
    PyObject *pModule = PyImport_Import(pName);
    PyObject *pDict = PyModule_GetDict(pModule);
    mPost = PyDict_GetItemString(pDict, "send_one_alarm");
    Py_XDECREF(pName);
    Py_XDECREF(pDict);
    Py_XDECREF(pModule);
}

ServerAlarmApi *ServerAlarmApi::GetInstance()
{
    if(m_pInstance == NULL)
        m_pInstance = new ServerAlarmApi();
    return m_pInstance;
}
void ServerAlarmApi::pyDecref()
{
    Py_DECREF(mPost);
    Py_Finalize();
}

// @description 发送一条Log
// @param logCode: Log code
// @param param: 动态参数
// @return 返回执行结果, 返回状态码,1000表示执行成功,否则表示执行失败。失败时打印错误信息
int ServerAlarmApi::sendOneAlarm(unsigned long long logCode, string param)
{
    if(!enabled)
    {
        std::cout<<"LogService: 0x"<<std::hex<<logCode<<": "<<param<<std::endl;
        return 0;
    }
    if(mInitFlag){
        std::cout << "ServerAlarmApi init failed" << endl;
        return -1;
    }
    int statusCode = 1001;
    try{
        PyGILState_STATE gstate;
        gstate = PyGILState_Ensure();
        PyObject *pArgs = PyTuple_New(2);
        sprintf(mBuf, "%016llX", logCode);
        PyTuple_SetItem(pArgs, 0, Py_BuildValue("s",mBuf));
        PyTuple_SetItem(pArgs, 1, Py_BuildValue("s",param.c_str()));
        PyObject *result = PyObject_CallObject(mPost, pArgs);
        Py_XDECREF(pArgs);
        if(result != NULL){
            PyArg_Parse(result, "i", &statusCode);// 解析失败返回0
            Py_XDECREF(result);
        }
        PyGILState_Release(gstate);
    }
    catch(...){
        std::cout << "PyArg_ParseTuple fail" << endl;
    }
    return statusCode;
}
// @description 发送一条Log
// @param logCode: Log code
// @return 返回执行结果, 返回状态码,1000表示执行成功,否则表示执行失败。失败时打印错误信息
int ServerAlarmApi::sendOneAlarm(unsigned long long logCode)
{
    if(!enabled)
    {
        std::cout<<"LogService: 0x"<<std::hex<<logCode<<std::endl;
        return 0;
    }
    if(mInitFlag){
        std::cout << "ServerAlarmApi init failed" << endl;
        return -1;
    }
    int statusCode = 1001;
    try{
        PyGILState_STATE gstate;
        gstate = PyGILState_Ensure();
        PyObject *pArgs = PyTuple_New(2);
        sprintf(mBuf, "%016llX", logCode);
        PyTuple_SetItem(pArgs, 0, Py_BuildValue("s",mBuf));
        PyTuple_SetItem(pArgs, 1, Py_BuildValue("s",""));
        PyObject *result = PyObject_CallObject(mPost, pArgs);
        Py_XDECREF(pArgs);
        if(result != NULL){
            PyArg_Parse(result, "i", &statusCode);// 解析失败返回0
            Py_XDECREF(result);
        }
        PyGILState_Release(gstate);
    }
    catch(...){
        std::cout << "PyArg_ParseTuple fail" << endl;
    }
    return statusCode;
}

void ServerAlarmApi::setEnable(bool enable_status)
{
    enabled = enable_status;
}


