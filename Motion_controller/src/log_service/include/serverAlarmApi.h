#ifndef ServerAlarmApi_H_2018_10_19_
#define ServerAlarmApi_H_2018_10_19_

#include <string>
#include <curl/curl.h>
#include <queue>
using  std::string;
using std::queue;
#define ALARM_SERVER_URL "http://0.0.0.0:9003"
#define CHUNCK_SIZE 256
//#define PRINT_ENABLE

struct MemoryStruct {
  char *memory;
  size_t size;
};

struct AlarmEvent {
    unsigned long long int code;
    struct timeval stamp;
    std::string param;
};

class ServerAlarmApi
{
private:
    ServerAlarmApi();
    struct MemoryStruct m_chunk_;
    pthread_t m_tid;
    char event_code_str_[32];  // 用于转换long long到string
    char time_str_[32];  // 字符串格式的时间
    bool enabled;
    static ServerAlarmApi *m_instance_ptr_;
    /**
	* @brief HTTP POST请求
	* @param strUrl 输入参数,请求的Url地址,如:http://www.baidu.com
	* @param strPost 输入参数,使用如下格式para1=val1¶2=val2&…
	* @param strResponse 输出参数,返回的内容
	* @return 返回是否Post成功
    */
    char *constructEventInfo(unsigned long long event_code, string event_param);
    void parseResponse(char *response);

public:
    bool m_loop;
    queue<char*> m_eventCodeQue;
    std::vector<AlarmEvent> alarm_fifo_;
    pthread_cond_t m_cond;
    pthread_mutex_t m_lock;
    pthread_mutex_t m_syncLock;
    static ServerAlarmApi *GetInstance();
    long getTimestamp();
    int post(const char *str_url, const char *str_post);
    void printDebugInfo(string debugInfo);
    void clearUp();
    int sendOneAlarm(unsigned long long event_code, string event_param);
    int sendOneAlarm(unsigned long long event_code);
    void setEnable(bool enable_status);
};
#endif
