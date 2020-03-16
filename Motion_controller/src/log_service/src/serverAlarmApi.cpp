#include "log_service_cJSON.h"
#include "serverAlarmApi.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdexcept>
#include <string.h>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
using namespace std;

ServerAlarmApi *ServerAlarmApi::m_instance_ptr_ = NULL;
void* threadFun(void *arg);
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    char *ptr = (char*)(realloc(mem->memory, mem->size + realsize + 1));
    if(ptr == NULL) {
        /* out of memory! */
        printf("\033[31;40m%s\033[0m\r\n", "not enough memory (realloc returned NULL)\n");
        return 0;
    }

    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;
    return realsize;
}

ServerAlarmApi::ServerAlarmApi():enabled(false)
{
    m_chunk_.memory = (char*)malloc(256);  /* will be grown as needed by the realloc above */
    m_chunk_.size = 0;    /* no data at this point */
    CURLcode res  = curl_global_init(CURL_GLOBAL_ALL);
    if(res != CURLE_OK) {
        fprintf(stderr, "curl_global_init() failed: %s\n", curl_easy_strerror(res));
    }

    m_loop = true;
    pthread_mutex_init(&m_lock, NULL);
    pthread_cond_init(&m_cond, NULL);
    pthread_mutex_init(&m_syncLock, NULL);
    if(pthread_create(&m_tid,NULL,threadFun,this)!=0)
    {
        cout<<"create thread failed!"<<endl;
        return;
    }
}

ServerAlarmApi *ServerAlarmApi::GetInstance()
{
    if(m_instance_ptr_ == NULL)
        m_instance_ptr_ = new ServerAlarmApi();

    return m_instance_ptr_;
}

void ServerAlarmApi::clearUp()
{
    curl_global_cleanup();
    free(m_chunk_.memory);
    m_loop = false;
}
void ServerAlarmApi::parseResponse(char *response)
{
    cJSON *res_root = cJSON_Parse(response);
    if (NULL == res_root) {
        printf("\033[31;40m%s\033[0m\r\n", "json pack into cjson error...");
        return;
    }
    cJSON *res_status_code = cJSON_GetObjectItem(res_root , "statusCode");
    if(cJSON_Number ==  res_status_code->type)
    {
        if (1000 != res_status_code->valueint)
        {
            printf("\033[31;40m%s\033[0m\r\n", "controller send log to service failed!");
            cJSON *res_param = cJSON_GetObjectItem(res_root , "param");
            cJSON *res_result = cJSON_GetObjectItem(res_param , "result");
            if(cJSON_String == res_result->type)
                printf("\033[31;40m%s\033[0m\r\n", res_result->valuestring);
        }
    }
    cJSON_Delete(res_root);
}

int ServerAlarmApi::post(const char *str_url, const char *str_post)
{
    CURL *curl = curl_easy_init();
    if(!curl) {
        fprintf(stderr, "curl_easy_init() failed\n");
        curl_global_cleanup();
        return int(CURLE_FAILED_INIT);
    }

    /* First set the URL that is about to receive our POST. */
    curl_easy_setopt(curl, CURLOPT_URL, str_url);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS/*CURLOPT_HTTPPOST*/, str_post);
    curl_easy_setopt(curl, CURLOPT_HEADER, 0L); /* include header */
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);  //得到请求结果后的回调函数
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &m_chunk_);

    /* Now, we should be making a zero byte POST request */
    CURLcode res = curl_easy_perform(curl);
    /* always cleanup */
    curl_easy_cleanup(curl);
    if(res != CURLE_OK)
        printf("\033[31;40mcurl_easy_perform() failed: %s\033[0m\r\n", curl_easy_strerror(res));
    else
    {
        long responseCode = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &responseCode);
        if (responseCode < 200 || responseCode >= 300)
            printf("\033[31;40mresponse Error:%d\033[0m\r\n", int(responseCode));
        else
            parseResponse(m_chunk_.memory);
    }
    m_chunk_.size = 0;
    return int(res);
}

char *ServerAlarmApi::constructEventInfo(unsigned long long event_code, string event_param)
{
    cJSON *root;
    cJSON *param;
    root = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "cmd", cJSON_CreateString("alarm/append_one_alarm"));
    cJSON_AddItemToObject(root, "param", param = cJSON_CreateObject());
    // 时间
    time_t t = time(0);
    strftime(time_str_, sizeof(time_str_), "%Y-%m-%d %H:%M:%S", localtime(&t)); //年-月-日 时:分:秒
    cJSON_AddItemToObject(param, "time", cJSON_CreateString(time_str_));
    // 事件编码
    sprintf(event_code_str_, "%016llX", event_code/*0x0001000100A70000*/);
    cJSON_AddItemToObject(param, "code", cJSON_CreateString(event_code_str_));
    // 事件参数
    cJSON *detailEn = cJSON_CreateArray();
    cJSON *detailCn = cJSON_CreateArray();
    if(event_param != ""){
        cJSON_AddItemToArray(detailEn, cJSON_CreateString(event_param.c_str()));
        cJSON_AddItemToArray(detailCn, cJSON_CreateString(event_param.c_str()));
    }
    cJSON_AddItemToObject(param, "detailEn", detailEn);
    cJSON_AddItemToObject(param, "detailCn", detailCn);

    char *tmpValue = cJSON_Print(root);
    cJSON_Delete(root);
    return tmpValue;
}

int ServerAlarmApi::sendOneAlarm(unsigned long long event_code, std::string event_param)
{
    if(!enabled)
    {
        //std::cout<<"LogService: 0x"<<std::hex<<log_code<<std::endl;
        return 0;
    }
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    AlarmEvent event;
    event.code = event_code;
    event.stamp = time_now;
    event.param = event_param;
    // 保护m_eventCodeQue
    pthread_mutex_lock(&m_lock);
    char *tmpValue = constructEventInfo(event_code, event_param);
    m_eventCodeQue.push(tmpValue);
    alarm_fifo_.push_back(event);
    pthread_mutex_unlock(&m_lock);
    // 通知发送线程
    pthread_mutex_unlock(&m_syncLock);
    return 1;

}
int ServerAlarmApi::sendOneAlarm(unsigned long long event_code)
{
    return sendOneAlarm(event_code, "");
}
long ServerAlarmApi::getTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000 + tv.tv_usec/1000;
}
void ServerAlarmApi::printDebugInfo(string debugInfo)
{
#ifdef PRINT_ENABLE
    long timestamp = getTimestamp();
    cout << timestamp << ":" << debugInfo << endl;
#endif
}

// @description 线程函数,用于向报警服务发送报警
// @param *arg: 线程入口参数
// @return NULL
void* threadFun(void *arg)
{
    // 等待事件超时时间
    char temp[32] = {0};
    char buffer[512] = {0};
    string string_cache;
    std::ofstream log_file;
    std::ofstream alarm_file;
    ServerAlarmApi *pServerApi = (ServerAlarmApi *)arg;
    boost::filesystem::path path = "/root/event/";

    if (!boost::filesystem::is_directory(path))
    {
        boost::filesystem::create_directories(path);
    }

    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    strftime(temp, 64, "%Y%m%d%H%M%S", local);
    srand(int(time(0)));
    sprintf(buffer, "/root/event/Alarm_%s_%x.log", temp, rand());

    alarm_file.open(buffer, std::ios::app);
    log_file.open("/root/Alarm.history", std::ios::app);
    log_file << buffer << endl;
    log_file.close();

    while(pServerApi->m_loop)
    {
        try
        {
            // 阻塞等待
            pthread_mutex_lock(&pServerApi->m_syncLock);  // lock
            pServerApi->printDebugInfo("开始发送报警");
            // 循环队列
            for(;;){

                if(pServerApi->m_eventCodeQue.empty())
                {
                    pServerApi->printDebugInfo("报警队列为空");
                    break;
                } else {
//                    cout <<"报警队列的长度:" << pServerApi->m_eventCodeQue.size() << endl;;
                }
                // 从队列中pop数据
                char *eventCodeStr=NULL;
                pthread_mutex_lock(&pServerApi->m_lock);  // lock
                eventCodeStr=pServerApi->m_eventCodeQue.front();
                pServerApi->m_eventCodeQue.pop();
                pthread_mutex_unlock(&pServerApi->m_lock); // unlock
                // 发送报警
                if(eventCodeStr)
                {
                    pServerApi->post(ALARM_SERVER_URL, eventCodeStr);
                    free(eventCodeStr);
                    pServerApi->printDebugInfo("发送一条完毕");
                }
            }

            if (!pServerApi->alarm_fifo_.empty())
            {
                string_cache.clear();
                pthread_mutex_lock(&pServerApi->m_lock);  // lock

                for (vector<AlarmEvent>::iterator it = pServerApi->alarm_fifo_.begin(); it != pServerApi->alarm_fifo_.end(); ++it)
                {
                    sprintf(buffer, "[%ld.%06ld]0x%016llx,%s", it->stamp.tv_sec, it->stamp.tv_usec, it->code, it->param.c_str());
                    string_cache = string_cache + buffer + '\n';
                }

                pServerApi->alarm_fifo_.clear();
                pthread_mutex_unlock(&pServerApi->m_lock); // unlock
                alarm_file << string_cache;
                alarm_file.flush();
            }
        }
        catch(...){
            cout<< "threadFun发生异常" << endl;
        }
    }

    alarm_file.close();
    cout<< "ServerAlarmApi exit thread" << endl;
    return NULL;
}

void ServerAlarmApi::setEnable(bool enable_status)
{
    enabled = enable_status;
}

