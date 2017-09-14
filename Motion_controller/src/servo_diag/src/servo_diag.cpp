#include <string.h>
#include <servo_diag.h>
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
#include <vector>
#include <boost/filesystem.hpp>
#include <signal.h>  
#include <servo_diag_version.h>
#include <ros/ros.h>
using namespace fst_controller;

fst_comm_interface::CommInterface ServoDiag::comm_;

int ServoDiag::exit_flag_ = 0;

ServoDiag::ServoDiag(void)
{

}


ServoDiag::~ServoDiag(void)
{

}


ERROR_CODE_TYPE ServoDiag::initComm(const char *ip_address, int port)
{
    char ip_str[30];

    snprintf(ip_str,30,"%s:%d", ip_address, port);
    std::cout<<ip_str<<std::endl;
    ERROR_CODE_TYPE err = comm_.createChannel(COMM_REP, COMM_TCP, ip_str);
    if (err == CREATE_CHANNEL_FAIL)
    {
        printf("Error when server createChannel.\n");
    }
    return err;
}

void ServoDiag::servoDiagThread(Servconf *servconf,
                                     DataMonitor *monitor,
                                     ServoService* service)
{

        
    ServoCommand_Pkg_t pkg;
    ServoService::initComm("test");
    while(comm_.recv(&pkg, sizeof(pkg), COMM_WAIT)!=RECV_MSG_FAIL)
    {
        switch(pkg.id)
        {
            case PC_STARTRECORD:
            {
                std::vector<int> t_list;
                int i;
                unsigned int ss_size;
                service->startLog(*(int*)&pkg.data[0],&pkg.data[4],t_list);
                DataMonitor::startMonitor(monitor,t_list,ss_size);
                *(unsigned int *)&pkg.data[0] = ss_size;
                for(i = 0;i<(int)t_list.size()&&i<SERVO_CMD_SEG_LENGTH/4;i++)
                {
                    *(int*)&pkg.data[i*4+4] = t_list[i];
                }

                break;
            }
            case PC_STOPRECORD:
            {
                service->stopLog();
                DataMonitor::stopMonitor(monitor);
                break;
            }    
            case PC_READINT:
            {
                ERROR_CODE_TYPE err = service->readIntVar(*(int*)&pkg.data[0],&pkg.data[4],(int*)&pkg.data[0]);
                if(FST_SUCCESS!=err)  
                {
                    std::cout<<"Read Int failed!"<<std::endl;
                }
                break;
            }
            case PC_SETTRIGGER:
            {
                ERROR_CODE_TYPE err = service->setTrig(&pkg.data[8],*(int*)&pkg.data[0],(int*)&pkg.data[0]);
                DataMonitor::setSnapshotSize(monitor,*(unsigned int*)&pkg.data[4]);
                if(FST_SUCCESS!=err)  
                {
                    std::cout<<"Set trigger failed!"<<std::endl;
                }
                break;
            }
            case PC_READSERVODTC:
            {
                ERROR_CODE_TYPE err = service->readErrCode((int)(SERVO_CMD_SEG_LENGTH-sizeof(int))/4,(int*)&pkg.data[4],(int*)&pkg.data[0]);
                if(FST_SUCCESS!=err)  
                {
                    std::cout<<"Read Err Code failed!"<<std::endl;
                }
                break;
            }
            case PC_READSERVOPARA:
            {
                //read param
                servconf->getConf(*(unsigned int*)&pkg.data[0],&pkg.data[8],*(int*)&pkg.data[4]);
                
                break;
            }          

            case PC_WRITESERVOPARA:
            {
                //write param
                servconf->setConf(*(unsigned int*)&pkg.data[0],&pkg.data[8],*(int*)&pkg.data[4]);
                servconf->downloadConf(*service,*(unsigned int*)&pkg.data[0],*(int*)&pkg.data[4]);
                break;
            }     
            case PC_SAVESERVOPARA:
            {
                //save param
                servconf->saveConf();
                break;
            }    
            case PC_SERVOCMD:
            {
                ERROR_CODE_TYPE err = service->servoCmd(*(int*)&pkg.data[0],&pkg.data[4],1020,&pkg.data[4],1020);
                if(FST_SUCCESS!=err)  
                {
                    std::cout<<"Servo CMD failed!"<<std::endl;
                }
                break;
            }             
            default:
                pkg.id = 0;
                break;
            
        } 
        comm_.send(&pkg, sizeof(pkg), COMM_DONTWAIT);
    }
  
}

void ServoDiag::sigHandler( int sig)
{
       if(sig == SIGINT){
              exit_flag_ = 1;
       }
}

int main(int argc, char** argv)
{
    char *ip;
    char buf[1024] = { 0 };
    ros::init(argc, argv, "servo_diag");	
    if (!fst_comm_interface::CommInterface::getLocalIP(&ip)) return 0;
    std::cout<<"Servo Diag Version:"<<servo_diag_VERSION_MAJOR<<"."<<servo_diag_VERSION_MINOR<<"."<<servo_diag_VERSION_PATCH<<std::endl;
    signal(SIGINT, ServoDiag::sigHandler);
    ServoDiag::initComm(ip,ServoDiag::SERVODIAG_PORT);

    readlink("/proc/self/exe" , buf , sizeof(buf));
    boost::filesystem::path pa(buf);
    std::string conffile(pa.parent_path().parent_path().parent_path().string()+"/share/configuration/machine/servo_param.yaml");
    Servconf* pconf = new Servconf(conffile);
    
    DataMonitor* pmonitor = new DataMonitor(ip,ServoDiag::DATAMONITOR_PORT);
    pmonitor->initDataMonitor();
    ServoService* pservice = new ServoService();
    pservice->initComm("test");
    if(1 == argc)
        pconf->initDownloadConf(*pservice);
    else
    {
        if(0==strcmp(argv[1],"upload"))
            pconf->initConfFile(*pservice);
    }
    boost::thread thrd_diag(boost::bind(ServoDiag::servoDiagThread, pconf,pmonitor,pservice));
    thrd_diag.detach();
    while(!ServoDiag::exit_flag_)
    {
        sleep(1);
    }
    pservice->stopLog();
    std::cout<<std::endl<<"ctrl+c has been keydownd, and log is off"<<std::endl;
    return 0;
}


