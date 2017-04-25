#include <string.h>
#include <servo_diag.h>
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
#include <vector>
#include <boost/filesystem.hpp>
#include <signal.h>  

using namespace fst_controller;

fst_comm_interface::CommInterface ServoDiag::comm_;

int ServoDiag::exit_flag_ = 0;

ServoDiag::ServoDiag(void)
{

}


ServoDiag::~ServoDiag(void)
{

}


ERROR_CODE_TYPE ServoDiag::InitComm(const char *ip_address, int port)
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

void ServoDiag::ServoDiag_Thread(Servconf *servconf,
                                     DataMonitor *monitor,
                                     ServoService* service)
{

        
    ServoCommand_Pkg_t pkg;
    ServoService::InitComm("test");
    while(comm_.recv(&pkg, sizeof(pkg), COMM_WAIT)!=RECV_MSG_FAIL)
    {
        switch(pkg.id)
        {
            case 0x01:
            {
                std::vector<int> t_list;
                int i;
                service->StartLog(*(int*)&pkg.data[0],&pkg.data[4],t_list);
                DataMonitor::StartMonitor(monitor,t_list);
                
                for(i = 0;i<t_list.size()&&i<SERVO_CMD_SEG_LENGTH/4;i++)
                {
                    *(int*)&pkg.data[i*4] = t_list[i];
                }

                break;
            }
            case 0x02:
            {
                service->StopLog();
                DataMonitor::StopMonitor(monitor);
                break;
            }    
            case 0x03:
            {
                ERROR_CODE_TYPE err = service->ReadIntVar(*(int*)&pkg.data[0],&pkg.data[4],(int*)&pkg.data[0]);
                if(FST_SUCCESS!=err)  std::cout<<"Read Int failed!"<<std::endl;
                break;
            }
            case 0x04:
            {
                ERROR_CODE_TYPE err = service->SetTrig(&pkg.data[4],*(int*)&pkg.data[0]);
                if(FST_SUCCESS!=err)  std::cout<<"Set trigger failed!"<<std::endl;
                break;
            }
            case 0x11:
            {
                //read param
                servconf->Getconf(*(unsigned int*)&pkg.data[0],&pkg.data[8],*(int*)&pkg.data[4]);
                
                break;
            }          

            case 0x12:
            {
                //write param
                servconf->Setconf(*(unsigned int*)&pkg.data[0],&pkg.data[8],*(int*)&pkg.data[4]);
                servconf->DownloadConf(*service,*(unsigned int*)&pkg.data[0],*(int*)&pkg.data[4]);
                break;
            }     
            case 0x20:
            {
                ERROR_CODE_TYPE err = service->ServoCMD(*(int*)&pkg.data[0],&pkg.data[4],1020,&pkg.data[4],1020);
                if(FST_SUCCESS!=err)  std::cout<<"Servo CMD failed!"<<std::endl;
                break;
            }             
            default:
                pkg.id = 0;
                break;
            
        }
        comm_.send(&pkg, sizeof(pkg), COMM_DONTWAIT);
    }
  
}

void ServoDiag::Sig_handler( int sig)
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
    signal(SIGINT, ServoDiag::Sig_handler);
    ServoDiag::InitComm(ip,5558);

    readlink("/proc/self/exe" , buf , sizeof(buf));
    boost::filesystem::path pa(buf);
    std::string conffile(pa.parent_path().string()+"/cfg/servo_param.yaml");
    Servconf* pconf = new Servconf(conffile);
    
    DataMonitor* pmonitor = new DataMonitor(ip,5559);
    pmonitor->initDataMonitor();
    ServoService* pservice = new ServoService();
    pservice->InitComm("test");
    if(1 == argc)
        pconf->InitDownloadConf(*pservice);
    else
    {
        if(0==strcmp(argv[1],"upload"))
            pconf->InitConfFile(*pservice);
    }
    boost::thread thrd_diag(boost::bind(ServoDiag::ServoDiag_Thread, pconf,pmonitor,pservice));
    thrd_diag.detach();
    while(!ServoDiag::exit_flag_)
    {
        sleep(1);
    }
    pservice->StopLog();
    std::cout<<std::endl<<"ctrl+c has been keydownd, and log is off"<<std::endl;
    return 0;
}


