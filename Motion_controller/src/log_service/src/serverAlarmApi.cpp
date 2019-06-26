#include "serverAlarmApi.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdexcept>
#include <string.h>
#include <iostream>
using namespace std;

ServerAlarmApi *ServerAlarmApi::m_instance_ptr_ = NULL;

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

ServerAlarmApi::ServerAlarmApi():
    enabled(false)
{
    char *xx = (char*)malloc(256);
    m_chunk_.memory = (char*)malloc(256);  /* will be grown as needed by the realloc above */
    m_chunk_.size = 0;    /* no data at this point */
    CURLcode res  = curl_global_init(CURL_GLOBAL_ALL);
    if(res != CURLE_OK) {
        fprintf(stderr, "curl_global_init() failed: %s\n", curl_easy_strerror(res));
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
            printf("response: %s. res_status_code=%d\n", response, res_status_code->valueint);
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
    cJSON *cmd;
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

    if (event_param == "")
    {
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
        //std::cout<<"LogService: 0x"<<std::hex<<log_code<<": "<<param<<std::endl;
        return 0;
    }
    try
    {
        char *tmpValue = constructEventInfo(event_code, event_param);
        int ret = post(ALARM_SERVER_URL, tmpValue);
        free(tmpValue);
        return int(ret);
    }
    catch(runtime_error& me)
    {
        cout<<me.what();
    }
    return 1;

}
int ServerAlarmApi::sendOneAlarm(unsigned long long event_code)
{
    if(!enabled)
    {
        //std::cout<<"LogService: 0x"<<std::hex<<log_code<<std::endl;
        return 0;
    }
    char *tmpValue = constructEventInfo(event_code, "");
    int ret = post(ALARM_SERVER_URL, tmpValue);
    free(tmpValue);
    return int(ret);
}

void ServerAlarmApi::setEnable(bool enable_status)
{
    enabled = enable_status;
}
