#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/ws.h>
#include <fstream>
#include <vector>
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "protoc.h"
#include <time.h>
#include "tp_comm_test.h"


#define MAX_REQ_BUFFER_SIZE     (65535)
using namespace std;
std::ifstream online_trajectory_file_;

 void file_to_string(vector<string> &record, const string& line, char delimiter);
 float string_to_float(string str);
 void file_to_string(vector<string> &record, const string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax = line.length();
    string curstring;
    record.clear();
    while (linepos<linemax)
    {
        c = line[linepos];
        if (isdigit(c) || c=='.' || c == '-')
        {
            curstring+=c;
        }
        else if(c==delimiter && curstring.size())
        {
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
    return;
}
 float string_to_float(string str){
    int i=0,len=str.length();
    float sum = 0;
    bool pn_flag=1;
    if(str[0] == '-')
    {pn_flag = 0;}
    else
    {sum = sum*10 + str[0] - '0';}
    ++i;
    while(i<len)
    {
        if(str[i]=='.') break;
        sum = sum*10 + str[i] - '0';
        ++i;
    }
    ++i;
    float t=1,d=1;
    while (i<len)
    {
        d*=0.1;
        t=str[i]-'0';
        sum+=t*d;
        ++i;
    }
    if(pn_flag)
        return sum;
    else
        return -1.0*sum;
}
//用于在线路径下发测试
int readJointTrajectoryFile(const std::string &online_trajectory_filePath, vector<vector<float> >&trajArr)
{
    if (online_trajectory_file_.is_open())
    {
        online_trajectory_file_.close();
    }
    cout << "[debug info]离线文件路径及名称: "<< online_trajectory_filePath << endl;
    online_trajectory_file_.open(online_trajectory_filePath.c_str());
    if (!online_trajectory_file_.is_open())
    {
        printf("mc_offline_traj","Fail to open offline trajectory file");
        return 0x0001000400A903FD;
    }
    vector<float> data_line;
    vector<string> row;
    string line;
    int line_cnt=0;//数据行计数
    //getline(online_trajectory_file_, line);//跳过第一行表头
    while (getline(online_trajectory_file_, line) && online_trajectory_file_.good())//逐行读取
    {
        file_to_string(row, line,' ');//把line里的单元格数字字符提取出来,' '为单元格分隔符
        line_cnt++;
		//printf("[%3d] ",line_cnt);
        for(int i=0,leng=row.size();i<leng;i++)
        {
            data_line.push_back(string_to_float(row[i]));
            //printf("%f ",data_line[i]);
        }
        //printf("\n");
        trajArr.push_back(data_line);
        data_line.clear();    
    }
    online_trajectory_file_.close();
    return 0;
}

int main(int argc, char** argv)
{
    TpCommTest test;
    if (!test.initRpcSocket())
    {
        cout << "Request : socket init failed" << endl;
        return -1;
    }
    if(argc < 4)
    {
        cout << "parameter < 4. [trajectoryFileName] [sendCnt] [sendInterval]" << endl;
        return 0;
    }
    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;
    int datalineCnt = 0;
    string trajectory_filePath = "/root/robot_data/trajectory/";
    trajectory_filePath += argv[1];
    unsigned int hash_value = 0x00008A31;//设置在线运动路径点数据RPC
    vector<vector<float>> trajArr;//二维数组暂存读入的数据
    double twentyPoint[480]={0};
    int  sendCnt = atoi(argv[2]);//发送次数
    int sendInterval = atoi(argv[3]);//发送间隔

    #if 1
        RequestMessageType_Int32_DoubleList msg;
        ResponseMessageType_Uint64 recv_msg;
        msg.data1.data = 0;
        msg.data2.data_count=0;
        msg.header.time_stamp = 122;
            msg.property.authority = Comm_Authority_TP_SIMMULATOR;
            if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_DoubleList_fields, buf, buf_size))
            {
                cout << "Request : encode buf failed" << endl;
                return -1;
            }
            if (!test.sendRequestBuf(buf, buf_size))
            {
                cout << "Request : send buf failed" << endl;
                return -1;
            }
            buf_size = MAX_REQ_BUFFER_SIZE;
            if (!test.recvResponseBuf(buf, buf_size))
            {
                cout << "Reply : recv buf failed, buf size = " << buf_size << endl;
                return -1;
            }
            
            unsigned int recv_hash = 0;
            if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
            {
                cout << "Reply : recv msg decode failed" << endl;
                return -1;
            }
            if (!test.checkHash(recv_hash, hash_value))
            {
                cout << "Reply : hash error ,hash = " << recv_hash << endl;
                return -1;
            }
            cout << "Reply : msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
            cout << "Reply : msg.header.package_left = " << recv_msg.header.package_left << endl;
            cout << "Reply : msg.header.error_code = " << recv_msg.header.error_code << endl;
            cout << "Reply : msg.property.authority = " << recv_msg.property.authority << endl;
            cout << "Reply : msg.data.data = " <<hex<<recv_msg.data.data << endl;
    #else

    if(readJointTrajectoryFile(trajectory_filePath, trajArr) == 0)
    {
        RequestMessageType_Int32_DoubleList msg;
        ResponseMessageType_Uint64 recv_msg;
        printf("RequestMessageType_Int32_DoubleList size=%d <==============\n",sizeof(RequestMessageType_Int32_DoubleList));
        //double traj_point[240]={0.0};
        int j=0;
        for(j=0;j<sendCnt;j++)
        {
            msg.data1.data = 0;
            msg.data2.data_count=480;
            memset(twentyPoint,0,480*sizeof(float));
            for(int t=0;t<20;t++)//一次发送20个点
            {
                twentyPoint[t*24+0] = trajArr[j*20+t][0];
                twentyPoint[t*24+1] = trajArr[j*20+t][1];
                twentyPoint[t*24+2] = trajArr[j*20+t][2];
                twentyPoint[t*24+3] = trajArr[j*20+t][3];
                twentyPoint[t*24+4] = trajArr[j*20+t][4];
                twentyPoint[t*24+5] = trajArr[j*20+t][5];
                twentyPoint[t*24+6] = trajArr[j*20+t][6];
                twentyPoint[t*24+7] = trajArr[j*20+t][7];
                twentyPoint[t*24+8] = trajArr[j*20+t][8];
                twentyPoint[t*24+9] = trajArr[j*20+t][9];
                twentyPoint[t*24+10] = trajArr[j*20+t][10];
                twentyPoint[t*24+11] = trajArr[j*20+t][11];
                twentyPoint[t*24+12] = trajArr[j*20+t][12];
                twentyPoint[t*24+13] = trajArr[j*20+t][13];
                twentyPoint[t*24+14] = trajArr[j*20+t][14];
                twentyPoint[t*24+15] = trajArr[j*20+t][15];
                twentyPoint[t*24+16] = trajArr[j*20+t][16];
                twentyPoint[t*24+17] = trajArr[j*20+t][17];
                twentyPoint[t*24+18] = trajArr[j*20+t][18];
                twentyPoint[t*24+19] = trajArr[j*20+t][19];
                twentyPoint[t*24+20] = trajArr[j*20+t][20];
                twentyPoint[t*24+21] = trajArr[j*20+t][21];
                twentyPoint[t*24+22] = trajArr[j*20+t][22];
                twentyPoint[t*24+23] = trajArr[j*20+t][23];
            }
            memcpy(msg.data2.data, twentyPoint,480*sizeof(double));
            msg.header.time_stamp = 122;
            msg.property.authority = Comm_Authority_TP_SIMMULATOR;
            if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_DoubleList_fields, buf, buf_size))
            {
                cout << "Request : encode buf failed" << endl;
                return -1;
            }
            if (!test.sendRequestBuf(buf, buf_size))
            {
                cout << "Request : send buf failed" << endl;
                return -1;
            }
            buf_size = MAX_REQ_BUFFER_SIZE;
            if (!test.recvResponseBuf(buf, buf_size))
            {
                cout << "Reply : recv buf failed, buf size = " << buf_size << endl;
                return -1;
            }
            
            unsigned int recv_hash = 0;
            if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
            {
                cout << "Reply : recv msg decode failed" << endl;
                return -1;
            }
            if (!test.checkHash(recv_hash, hash_value))
            {
                cout << "Reply : hash error ,hash = " << recv_hash << endl;
                return -1;
            }

            cout << "Reply : msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
            cout << "Reply : msg.header.package_left = " << recv_msg.header.package_left << endl;
            cout << "Reply : msg.header.error_code = " << recv_msg.header.error_code << endl;
            cout << "Reply : msg.property.authority = " << recv_msg.property.authority << endl;
            cout << "Reply : msg.data.data = " <<hex<<recv_msg.data.data << endl;
            if(recv_msg.data.data != 0)
            {
                j--;
                //break;
            }
            usleep(sendInterval);
        }
    }
    #endif
    usleep(200000);

    return 0;
}


