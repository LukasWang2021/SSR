/**********************************************
File: service_test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test the service functions
Author: Feng.Wu 10-oct-2016
Modifier:
**********************************************/

#ifndef SERVICE_TEST_C_
#define SERVICE_TEST_C_

#include <string.h>
#include "comm_interface/comm_interface.h"
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>  
#include <boost/lexical_cast.hpp>
#include <servo_conf.h>
#include <serv_cfcomm.h>
#include "data_monitor.h"
#include <boost/thread.hpp>
#include <netinet/in.h>  /* For htonl and ntohl */
using namespace boost::property_tree;

const int SERVO_CONF_LENGTH = 4096;
const int SERVO_DATA_SEG_LENGTH = 512;


template <typename ElemT>
struct HexTo {
    ElemT value;
    operator ElemT() const {return value;}
    friend std::istream& operator>>(std::istream& in, HexTo& out) {
        in >> std::hex >> out.value;
        return in;
    }
};

static ERROR_CODE_TYPE send(fst_comm_interface::CommInterface &comm,ServiceRequest* req)
{
    return comm.send(req, sizeof(ServiceRequest), COMM_DONTWAIT);
}

static int recv(fst_comm_interface::CommInterface &comm,ServiceResponse* resp)
{

    ERROR_CODE_TYPE err = comm.recv(resp, sizeof(ServiceResponse), COMM_DONTWAIT);
    return (0==err)?1:0;
}
void sendCmd2Core1(fst_comm_interface::CommInterface &comm,unsigned int cmd)
{
    ServiceRequest client_service_request;
    client_service_request.req_id = 0x01;
    client_service_request.req_buff[0] = (char)((0xFF)&&(cmd));
    client_service_request.req_buff[1] = (char)((0xFF)&&(cmd>>8));
    client_service_request.req_buff[2] = (char)((0xFF)&&(cmd>>16));
    client_service_request.req_buff[3] = (char)((0xFF)&&(cmd>>24));
    send(comm, &client_service_request);
    usleep(1000);
    ServiceResponse client_service_response;   
    recv(comm, &client_service_response);
    sleep(1);
}

void sendConfData(fst_comm_interface::CommInterface &comm,unsigned int id,std::vector<double> &val_vector)
{
    ServiceRequest client_service_request;
    client_service_request.req_id = 0x2D;
    unsigned int len = val_vector.size();
    memcpy(&client_service_request.req_buff[0],(char *)&id,sizeof(id));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request.req_buff[4],(char *)&len,sizeof(len));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[4])<<std::endl;
    for(int i = 0;i<val_vector.size();++i)
    {
        memcpy(&client_service_request.req_buff[8+i*8],(char *)&val_vector[i],8);
        std::cout<<*((double *)&client_service_request.req_buff[8+i*8])<<std::endl;
    }
    send(comm, &client_service_request);
    usleep(1000);
    ServiceResponse client_service_response;   
    recv(comm, &client_service_response);
    sleep(1);
}

void sendServoConfData(fst_comm_interface::CommInterface &comm,unsigned int addr,const char *data,int length)
{
    ServiceRequest client_service_request;
    if(length>(1024-8)) return;
    client_service_request.req_id = 0x24;

    memcpy(&client_service_request.req_buff[0],(char *)&addr,sizeof(addr));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request.req_buff[4],(char *)&length,sizeof(length));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[4])<<std::endl;

    memcpy(&client_service_request.req_buff[8],(char *)data,length);
    std::cout<<*((unsigned short *)&client_service_request.req_buff[8])<<std::endl;

    send(comm, &client_service_request);
    usleep(1000);
    ServiceResponse client_service_response;   
    recv(comm, &client_service_response);
    //sleep(1);
}

void settrig(fst_comm_interface::CommInterface &comm,const char *trigname,unsigned short ticks)
{
    ServiceRequest client_service_request;
    client_service_request.req_id = 0x40;
    unsigned int id = 0x10;
    memcpy(&client_service_request.req_buff[0],(char *)&id,sizeof(id));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request.req_buff[4],(char *)&ticks,sizeof(ticks));
    std::cout<<*((unsigned short *)&client_service_request.req_buff[4])<<std::endl;
    int len = strlen(trigname);
    memcpy(&client_service_request.req_buff[6],trigname,len+1);
    send(comm, &client_service_request);
    usleep(1000);
    ServiceResponse client_service_response;   
    recv(comm, &client_service_response);
    sleep(1);
}

void startlog(fst_comm_interface::CommInterface &comm,unsigned short size_of_varlist,const char *varlist,int * typelist)
{
    ServiceRequest client_service_request;
    ServiceResponse client_service_response;   
    client_service_request.req_id = 0x40;
    unsigned int id = 0x11;
    memcpy(&client_service_request.req_buff[0],(char *)&id,sizeof(id));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;

    memcpy(&client_service_request.req_buff[4],(char *)&size_of_varlist,sizeof(size_of_varlist));
    std::cout<<*((unsigned short *)&client_service_request.req_buff[4])<<std::endl;

    memcpy(&client_service_request.req_buff[6],varlist,1024 - 6 - 1);
    //recv(comm, &client_service_response); 
    send(comm, &client_service_request);
    usleep(1000);
    int cnt = 0;
    int res = recv(comm, &client_service_response);
    while(res<1)
    {
        usleep(1000);
        if(++cnt>3) 
            { std::cout<<"fail to get respose"<<std::endl; break;}
        res = recv(comm, &client_service_response);
    }

    if(res>0)
    {
       int i;
       for (i=0;i<size_of_varlist;++i)
       {
           typelist[i] = *(int *)&client_service_response.res_buff[i*4];
           std::cout<<typelist[i]<<std::endl;
       }
       //std::cout<<i<<std::endl;
       
    }
    //sleep(1);
}

void stoplog(fst_comm_interface::CommInterface &comm)
{
    ServiceRequest client_service_request;
    client_service_request.req_id = 0x40;
    unsigned int id = 0x12;
    memcpy(&client_service_request.req_buff[0],(char *)&id,sizeof(id));
    send(comm, &client_service_request);
    usleep(1000);
    ServiceResponse client_service_response;   
    recv(comm, &client_service_response);
    sleep(1);
}

int recServoConfData(fst_comm_interface::CommInterface &comm,unsigned int addr,char *data,int length)
{
    ServiceRequest client_service_request;
    if(length>(1024-8)) return 0;
    client_service_request.req_id = 0x14;

    memcpy(&client_service_request.req_buff[0],(char *)&addr,sizeof(addr));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request.req_buff[4],(char *)&length,sizeof(length));
    std::cout<<*((unsigned int *)&client_service_request.req_buff[4])<<std::endl;

    ServiceResponse client_service_response; 
    //recv(comm, &client_service_response);  //clear response buf, but maybe there are still response in Send Process's queue
    send(comm, &client_service_request);
    usleep(1000);
    int cnt = 0;
    int res = recv(comm, &client_service_response);
    while(res<1)
    {
        usleep(1000);
        if(++cnt>3) break;
        res = recv(comm, &client_service_response);
    }

    if(res>0)
    {
       memcpy((char *)data,&client_service_response.res_buff[8],length);
       //std::cout<<*((char *)data)<<std::endl;
    }
    //sleep(1);
    return res;
}

int ParseConfFromFile(const char* filename,ptree &pt)
{
    std::ifstream is;
    is.open (filename, std::ios::binary|std::ios::in);
    try{      
        read_json(is, pt);  
    }  
    catch(ptree_error & e) {  
        return 1;   
    }  
    is.close();
    return 0;
}

template<typename T>
int getValueFromPt(ptree &pt,const char* key,T &data)
{
  try{  
      data = pt.get<T>(key);   
  }  
  catch (ptree_error & e)  
  {  
      return -1;  
  }  
  return 0;
}

template<typename T>
int getVectorFromPt(ptree &pt,const char* key,std::vector<T> &val_vector)
{
    int res = 0;
    T data;
    BOOST_FOREACH(ptree::value_type &v, pt)  
    {  
        ptree p = v.second;  
        try{  
            data = p.get<T>(key);   
        }  
        catch (ptree_error & e)  
        {  
            res = -1;  
            break;
        } 
	if(0==res)
        {
            val_vector.push_back(data);
        }
    }  
    return res;
}


void Download_servo_data(fst_comm_interface::CommInterface &comm,const std::string &filename)
{
    fst_controller::Servconf servo_conf(filename,SERVO_CONF_LENGTH);
//    servo_conf.Setconf(0,defservo,SERVO_CONF_LENGTH);
//    servo_conf.Saveconf(servofile.c_str());
    char *data = new char[SERVO_DATA_SEG_LENGTH];
    for(unsigned int addr = 0;addr<SERVO_CONF_LENGTH;)
    {
        
        servo_conf.Getconf(addr, data,SERVO_DATA_SEG_LENGTH);
        sendServoConfData(comm,addr,data,SERVO_DATA_SEG_LENGTH);
        addr += SERVO_DATA_SEG_LENGTH;
    }
    delete [] data;
}


void Upload_servo_data(fst_comm_interface::CommInterface &comm,const std::string &filename)
{
    fst_controller::Servconf servo_conf(SERVO_CONF_LENGTH);
//    servo_conf.Setconf(0,defservo,SERVO_CONF_LENGTH);
//    servo_conf.Saveconf(servofile.c_str());
    char *data = new char[SERVO_DATA_SEG_LENGTH];
    for(unsigned int addr = 0;addr<SERVO_CONF_LENGTH;)
    {
        int res = recServoConfData(comm,addr,data,SERVO_DATA_SEG_LENGTH);
        if(res>0)
        {
            servo_conf.Setconf(addr, data,SERVO_DATA_SEG_LENGTH);
            addr += SERVO_DATA_SEG_LENGTH;
        }
        else
        {
            std::cout<<"Error happened when try to read servo data!"<<std::endl;
            break;
        }
    }
    delete [] data;
    servo_conf.Saveconf(filename.c_str());
}

void Download_JTAC_data(fst_comm_interface::CommInterface &comm,ptree &pt)
{
    ptree jtac_array = pt.get_child("jtac_data");
    BOOST_FOREACH(ptree::value_type &v, jtac_array)  
    {  
        ptree p = v.second;  
        std::vector<double> val_vector;
        std::string idstr = p.get<std::string>("dataid");  
        std::cout<<idstr.c_str()<<std::endl;
        unsigned int dataid = boost::lexical_cast<HexTo<unsigned int> >(idstr.c_str());
        //std::cout<<dataid<<std::endl;
	getVectorFromPt(p.get_child("data"),"value",val_vector);
        //for(int i = 0;i<val_vector.size();i++)
        //{ std::cout<<val_vector[i]<<std::endl;}
	sendConfData(comm,dataid,val_vector);
    } 
}


int fifo_loop(std::ofstream &os,const int * typelist)
{
    LOG_RECORD_T record;

    while(getRecord(&record)>=0)
    {
        int i;
        os << (int)record.time_flag;
        //printf("%d",record.time_flag);
        for (i = 0;i<record.length;++i)
        {
            switch(typelist[i])
            {
                case 0:
                case 2:
                case 4:
                {
                    os << "," << *(unsigned int *)&(record.data[i]);
                    //printf(",%d",*(unsigned int *)&(record.data[i]));
                    break;
                }
                case 1:
                case 3:
                case 5:
                {
                    os << "," << *(int *)&(record.data[i]);
                    //printf(",%d",*(int *)&(record.data[i]));
                    break;
                }
                case 6:
                case 7:
                {
                    os << "," << *(double *)&(record.data[i]);
                    //printf(",%f",*(double *)&(record.data[i]));
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        //printf("\n");
        os << "\n";
    }

    return 0;
}

fst_comm_interface::CommInterface* p_comm;

void sig_handler( int sig)
{
       if(sig == SIGINT){
              stoplog(*p_comm);
              std::cout<<"\nctrl+c has been keydownd,and log is off"<<std::endl;
              exit(0);
       }
}
int main(int argc, char** argv)
{
    char buf[1024] = { 0 };

    fst_comm_interface::CommInterface comm;
    //std::string path(getcwd(NULL,0));//not work
    std::ofstream os;
    
  
    ERROR_CODE_TYPE result = comm.createChannel(COMM_REQ,COMM_IPC, "test");
    if (result == CREATE_CHANNEL_FAIL)
    {
        return 1;
    }
    p_comm = &comm;
    signal(SIGINT, sig_handler);
    int typelist[64];
    readlink("/proc/self/exe" , buf , sizeof(buf));
    boost::filesystem::path pa(buf);
    std::string conffile(pa.parent_path().string()+"/cfg/jtac.jason");
    ptree pt; 
    int i,res;
    int goon = 0;
    res = ParseConfFromFile(conffile.c_str(),pt);
    std::string logfile(pa.parent_path().string()+"/log/J.log");
    std::string servofile(pa.parent_path().string()+pt.get<std::string>("servo_conf"));
    if(argc>1)
    {
 
        if(0==strcmp(argv[1],"UpServo"))
        {
            Upload_servo_data(*p_comm, servofile);
        } 
        else if(0==strcmp(argv[1],"Reset"))
        {
            sendCmd2Core1(*p_comm,0);
        }
        else if(0==strcmp(argv[1],"Disable"))
        {
            sendCmd2Core1(*p_comm,1);
        }
        else if(0==strcmp(argv[1],"Download"))
        {
            Download_JTAC_data(*p_comm,pt);

            std::cout<<"Download Completed!"<<std::endl;
        }
        else if(0==strcmp(argv[1],"Servo"))
        {
            Download_servo_data(*p_comm, servofile);

            std::cout<<"Servo data download Completed!"<<std::endl;
        }

        else if(0==strcmp(argv[1],"Settrig"))
        { 
            if (argc<3)
            {
                std::cout<<"wrong arg number!"<<std::endl;
            }
            else
            {
                int k = boost::lexical_cast<int>(argv[2]);
                if (3==argc)
                    settrig(*p_comm,"",k);
                else
                    settrig(*p_comm,argv[3],k);
            }
                
        }
        else if(0==strcmp(argv[1],"Logon"))
        { 
            char buf[1024];
 
            int k = 2;
            for(int i = 0;(i<1024)&(k < argc);)
            {
                int len = strlen(argv[k]);
                memcpy(&buf[i],argv[k],len+1);
                //std::cout<<i<<":"<<argv[k]<<"::"<<&buf[i]<<std::endl;
                i = i + len + 1;
                k++;
            }
            //std::cout<<buf<<std::endl;
            os.open (logfile.c_str(), std::ios::binary|std::ios::trunc|std::ios::out);
            openMem(MEM_CORE);
            startlog(*p_comm,argc - 2,buf,typelist);
            goon = 2;
        }
        else if(0==strcmp(argv[1],"Logoff"))
        {
            stoplog(*p_comm);
        }
        else if(0==strcmp(argv[1],"help"))
        {
            std::cout<<"       UpServo : Upload servo configuration"<<std::endl;
            std::cout<<"       Reset : Reset Core1"<<std::endl;
            std::cout<<"       Disable : Disable Core1"<<std::endl;
            std::cout<<"       Download : Download JTAC Parameter"<<std::endl;
            std::cout<<"       Servo : Download Servo Parameter"<<std::endl;
            std::cout<<"       Logon : Start Modbus Monitor and logon monitor"<<std::endl;
        }
        else
            std::cout<<"Wrong argument! Try to run without argument or try \"help\" for help"<<std::endl;

    }

    //Download_servo_data(*p_comm, servofile);
    if (0 == goon)
        return 0;
    else
    {
        char *ip;
        if (!comm.getLocalIP(&ip)) return 0;
        fst_controller::DataMonitor* p_monitor = new fst_controller::DataMonitor(ip,5558);
        boost::thread thrd_moni(boost::bind(fst_controller::DataMonitor::pcComm_Thread, p_monitor));
        thrd_moni.detach();

        fst_controller::ServcfComm servcfcomm("0.0.0.0",102,SERVO_CONF_LENGTH,0);
        fst_controller::Servconf servo_conf(servofile,SERVO_CONF_LENGTH);
        unsigned int addr = 0;
        servo_conf.Getconf(addr, buf,SERVO_CONF_LENGTH);
        servcfcomm.SetRegisters(addr,buf,SERVO_CONF_LENGTH);
        int s = servcfcomm.Accept();
        int counter = 0;
        std::cout<<"Accepted"<<std::endl;
        
        while(1)
        {
            s = servcfcomm.HandleMsg(&addr,buf,SERVO_DATA_SEG_LENGTH);
            if(s>0)
            {
                counter = 0;
                std::cout<<"addr:"<<addr<<"size:"<<s<<std::endl;
                servo_conf.Setconf(addr, buf,s);
                servo_conf.Saveconf(servofile.c_str());
                sendServoConfData(*p_comm,addr,buf,s);
            }
            else
            {
                counter++;
                if(counter>10000)
                {
                    counter = 0;
                    std::cout<<"Ready to reconnect"<<std::endl;
                    servcfcomm.Accept();
                    std::cout<<"Accepted"<<std::endl;
                }
            }
            usleep(1000);
        }

        delete p_monitor;
    }
    return 0;
}

#endif

