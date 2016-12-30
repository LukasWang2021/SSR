/*************************************************************************
	> File Name: log_manager_server.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时01分09秒
 ************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <stdexcept>
#include <iostream>
#include <comm_interface/comm_interface.h>
#include "log_manager/log_manager_logger.h"
#include "log_manager/log_manager_server.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

#define LOCK    pthread_mutex_lock(&g_log_structure_ptr_queue_mutex);
#define UNLOCK  pthread_mutex_unlock(&g_log_structure_ptr_queue_mutex);

bool buildLogStructure(const char *channel_name);
void deleteLogStructure(const char *channel_name);
void do_io(std::ofstream& handle, vector<char*> segments);

void public_thread(void);
void receive_thread(void);
void io_thread(void);


bool g_running = true;
pthread_mutex_t g_log_structure_ptr_queue_mutex;
vector<fst_log::LogServerStruct*>   g_log_structure_ptr_queue;

bool buildLogStructure(const char *channel_name) {
    LOCK;
    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        if (g_log_structure_ptr_queue[i]->channel_name == channel_name) {
            UNLOCK;
            cout << " -Channel already exist, cannot construct a communication interface with name '"
                 << channel_name << "'" << endl;
            return false;
        }
    }
    UNLOCK;

    cout << " -Constructing log structure ..." << endl;
    char buf[256];
    memset(buf, 0, sizeof(buf));
    int length = readlink("/proc/self/exe", buf, sizeof(buf));
    if (length > 0 && length < sizeof(buf)) {
        boost::filesystem::path executable(buf);
        string abs_path = executable.parent_path().parent_path().parent_path().string() + "/log/";
                
        time_t time_now = time(NULL);
        tm *local = localtime(&time_now);
        memset(buf, 0, sizeof(buf));
        strftime(buf, 64, " %Y-%m-%d %H:%M:%S", local);
        string file_name = abs_path + channel_name + buf + ".log";

        fst_log::LogServerStruct *tmp_log_struct = NULL;
        try {
            tmp_log_struct = new fst_log::LogServerStruct;
        }
        catch (std::exception exc) {
            cout << " -Constructing log structure exception:" << exc.what() << endl;
            return false;
        }
        
        try {
            tmp_log_struct->comm_interface_ptr = new fst_comm_interface::CommInterface;
        }
        catch (std::exception exc) {
            cout << " -Constructing communication interface exception:" << exc.what() << endl;
            return false;
        }
        if (tmp_log_struct->comm_interface_ptr->createChannel(IPC_REP, channel_name) != 0) {
            cout << " -Cannot setup " << channel_name << " server" << endl;
            return false;
        }

        tmp_log_struct->open_flag = true;
        tmp_log_struct->channel_name = channel_name;
        tmp_log_struct->buffer_pool.reserve(10);
        tmp_log_struct->file_handle.open(file_name.c_str(), std::ios::app);
        if (tmp_log_struct->file_handle.is_open()) {
            LOCK;
            g_log_structure_ptr_queue.push_back(tmp_log_struct);
            UNLOCK;
            cout << " -Success!" << endl;
            return true;
        }
        else {
            cout << " -Cannot construct log file" << endl;
            return false;
        }
    }
    else {
        cout << " -Cannot construct log file" << endl;
        return false;
    }
}

void deleteLogStructure(const char *channel_name) {
    LOCK;
    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        if (g_log_structure_ptr_queue[i]->channel_name == channel_name) {
            cout << "Delete log structure: '" << g_log_structure_ptr_queue[i]->channel_name << "'" << endl;
            g_log_structure_ptr_queue[i]->open_flag = false;
            vector<char*> segments;
            segments.assign(g_log_structure_ptr_queue[i]->buffer_pool.begin(),
                            g_log_structure_ptr_queue[i]->buffer_pool.end());
            g_log_structure_ptr_queue[i]->buffer_pool.clear();
            std::ofstream &handle = g_log_structure_ptr_queue[i]->file_handle;
            UNLOCK;
            
            do_io(handle, segments);
            
            LOCK;
            g_log_structure_ptr_queue[i]->file_handle.close();
            delete g_log_structure_ptr_queue[i]->comm_interface_ptr;
            g_log_structure_ptr_queue.erase(g_log_structure_ptr_queue.begin() + i);
            UNLOCK;
            break;
        }
    }
    UNLOCK;
    cout << " -Success!" << endl;
}

void do_io(std::ofstream& handle, vector<char*> segments)
{
    for (int cnt = 0; cnt < segments.size(); ++cnt) {
        handle << segments[cnt];
        delete[] segments[cnt];
    }
}

void public_thread(void) {
    cout << "Constructing public server ..." << endl;
    fst_comm_interface::CommInterface server;
    if (server.createChannel(IPC_REP, "log_public") != 0) {
        cout << " -Cannot setup public server." << endl;
        g_running = false;
    }
    cout << " -Success!" << endl;

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    string channel_to_close = "";
    while (g_running) {
        usleep(50 * 1000);
        if (server.recv(buffer, sizeof(buffer), IPC_DONTWAIT) == 0) {
            cout << "A new log request received: '" << buffer << "'" << endl;
            if (buildLogStructure(buffer)) {
                if (server.send(buffer, sizeof(buffer), IPC_DONTWAIT) == 0) {
                    cout << " -Log structure ready, logging '" << buffer << "'" << endl;
                }
                else {
                    deleteLogStructure(buffer);
                    cout << " -Lost communication with remote log client, the log structure removed." << endl;
                }
            }
            else {
                cout << " -Cannot construct log structure" << endl;
            }
        }

        LOCK;
        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->open_flag == false) {
                channel_to_close = g_log_structure_ptr_queue[i]->channel_name;
                break;
            }
        }
        UNLOCK;

        if (channel_to_close != "") {
            deleteLogStructure(channel_to_close.c_str());
            channel_to_close = "";
        }
    }  // while (g_running)

    cout << " -Public thread terminated." << endl;
}

void receive_thread(void) {
    char recv_buffer[LOG_BUFFER_SIZE];
    memset(recv_buffer, 0, sizeof(recv_buffer));

    while (g_running) {
        usleep(5 * 1000);

        LOCK;
        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->open_flag == false) {
                continue;
            }

            if (g_log_structure_ptr_queue[i]->comm_interface_ptr->recv(recv_buffer, sizeof(recv_buffer), IPC_DONTWAIT) == 0) {
                // cout << "received" << endl;
                char *buf = new char[LOG_BUFFER_SIZE];
                memcpy(buf, recv_buffer, LOG_BUFFER_SIZE);

                if (buf[0] == '\33' && buf[6] == '\33') {
                    if (buf[1] == 'C' && buf[2] == 'L' && buf[3] == 'O' && buf[4] == 'S' && buf[5] == 'E' ) {
                        g_log_structure_ptr_queue[i]->open_flag = false;
                        continue;
                    }
                }

                g_log_structure_ptr_queue[i]->buffer_pool.push_back(buf);
            }
        }
        UNLOCK;
    }

    cout << " -Receive thread terminated." << endl;
}


void io_thread(void) {
    bool no_io_task = true;

    while(g_running) {
        // usleep(20 * 1000);
        LOCK;
        no_io_task = true;
        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                vector<char*> segments;
                segments.assign(g_log_structure_ptr_queue[i]->buffer_pool.begin(),
                                g_log_structure_ptr_queue[i]->buffer_pool.end());
                g_log_structure_ptr_queue[i]->buffer_pool.clear();
                std::ofstream &handle = g_log_structure_ptr_queue[i]->file_handle;
                UNLOCK;
                do_io(handle, segments);
                no_io_task = false;
                break;
            }
            /*
            while (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                char *segment = g_log_structure_ptr_queue[i]->buffer_pool.front();
                g_log_structure_ptr_queue[i]->buffer_pool.erase(g_log_structure_ptr_queue[i]->buffer_pool.begin());
                g_log_structure_ptr_queue[i]->file_handle << segment;
                delete[] segment;
            }
            */
        }
        if (no_io_task) {
            UNLOCK;
            usleep(50 * 1000);
        }
    }  // while (g_running)

    usleep(200 * 1000);
    cout << " -IO thread is logging the rest of the logs." << endl;
    LOCK;
    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        while (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                char *segment = g_log_structure_ptr_queue[i]->buffer_pool.front();
                g_log_structure_ptr_queue[i]->buffer_pool.erase(g_log_structure_ptr_queue[i]->buffer_pool.begin());
                g_log_structure_ptr_queue[i]->file_handle << segment;
                delete[] segment;
        }
        // g_log_structure_ptr_queue[i]->file_handle.close();
    }
    UNLOCK;

    while (g_log_structure_ptr_queue.size() > 0) {
        deleteLogStructure(g_log_structure_ptr_queue.front()->channel_name.c_str());
    }

    cout << " -IO thread terminated." << endl;
}

static void sigintHandle(int num)
{
    cout << "Interrupt request catched." << endl;
    g_running = false;
    usleep(500 * 1000);
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandle);
    pthread_mutex_init(&g_log_structure_ptr_queue_mutex, NULL);
    g_log_structure_ptr_queue.reserve(10);
    boost::thread log_public(&public_thread);
    boost::thread log_receive(&receive_thread);
    boost::thread log_io(&io_thread);
    
    log_public.join();
    log_receive.join();
    log_io.join();
   
    cout << "Log server exit." << endl;
    return 0;
}
