#include <string.h>
#include <servo_service.h>
#include <data_monitor.h>

using namespace fst_comm_interface;

CommInterface* fst_controller::ServoService::p_comm_;
int fst_controller::ServoService::counter_ = 0;
std::mutex fst_controller::ServoService::mtx_;

fst_controller::ServoService::ServoService(void)
{
    counter_++;
}


fst_controller::ServoService::~ServoService(void)
{
    counter_--;
    if ((counter_ <= 0) && (NULL != p_comm_))
        delete p_comm_;
}

ERROR_CODE_TYPE fst_controller::ServoService::InitComm(const char *channel)
{
    ERROR_CODE_TYPE err = FST_SUCCESS;
    p_comm_ = new CommInterface;
    if (NULL != p_comm_)
    {
        err = p_comm_->createChannel(COMM_REQ, COMM_IPC, channel);

        if (err == CREATE_CHANNEL_FAIL)
        {
            delete p_comm_;
            p_comm_ = NULL;
            printf("Error when server createChannel.\n");  
        }
    }
    else
        err = CREATE_CHANNEL_FAIL;
    return err;
}

ERROR_CODE_TYPE fst_controller::ServoService::SendNRecv(fst_controller::ServoService* serv)
{
    const ServiceRequest* req = &serv->client_service_request_;
    ServiceResponse* resp = &serv->client_service_response_;
    ERROR_CODE_TYPE err;
    if (NULL == p_comm_)
    {
        return CREATE_CHANNEL_FAIL;
    }
    mtx_.lock();
    err = p_comm_->send(req, sizeof(ServiceRequest), COMM_DONTWAIT);
    if(FST_SUCCESS == err)
    {
        int cnt = 0;
        do
        {
            cnt++;
            usleep(1000);
            err = p_comm_->recv(resp, sizeof(ServiceResponse), COMM_DONTWAIT);
            if(cnt>50) break;
        }while(FST_SUCCESS != err);
    }
    mtx_.unlock();
    
    return err;
}

ERROR_CODE_TYPE fst_controller::ServoService::StartLog(int size_of_varlist,const char *varlist,std::vector<int>& t_list)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x40;
    unsigned int id = 0x11;
    memcpy(&client_service_request_.req_buff[0],(char *)&id,sizeof(id));
    std::cout<<*((unsigned int *)&client_service_request_.req_buff[0])<<std::endl;

    memcpy(&client_service_request_.req_buff[4],(char *)&size_of_varlist,sizeof(size_of_varlist));
    std::cout<<*((unsigned short *)&client_service_request_.req_buff[4])<<std::endl;

    memcpy(&client_service_request_.req_buff[6],varlist,1024 - 6 - 1);
    std::cout<<"list size:"<<size_of_varlist<<std::endl;
    err = SendNRecv(this);
    t_list.clear();
    if(FST_SUCCESS == err)
    {
       int i;
       std::cout<<"type list:"<<std::endl;
       for (i=0;i<size_of_varlist;++i)
       {
           t_list.push_back(*(int *)&client_service_response_.res_buff[i*4]);
           std::cout<<t_list[i]<<std::endl;
       }        
    }
    return err;
}

ERROR_CODE_TYPE fst_controller::ServoService::StopLog(void)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x40;
    unsigned int id = 0x12;
    memcpy(&client_service_request_.req_buff[0],(char *)&id,sizeof(id));
    err = SendNRecv(this);
    return err;
}


ERROR_CODE_TYPE fst_controller::ServoService::DownloadParam(unsigned int addr,const char *data,int length)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x24;
    
    memcpy(&client_service_request_.req_buff[0],(char *)&addr,sizeof(addr));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request_.req_buff[4],(char *)&length,sizeof(length));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[4])<<std::endl;
    
    memcpy(&client_service_request_.req_buff[8],(char *)data,length);
    //std::cout<<*((unsigned short *)&client_service_request.req_buff[8])<<std::endl;
    
    err = SendNRecv(this);
    return err;
}


ERROR_CODE_TYPE fst_controller::ServoService::UploadParam(unsigned int addr,char *data,int& length)
{
    ERROR_CODE_TYPE err;
    if(length>SERVO_CONF_SEG) return 1;
    client_service_request_.req_id = 0x14;

    memcpy(&client_service_request_.req_buff[0],(char *)&addr,sizeof(addr));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request_.req_buff[4],(char *)&length,sizeof(length));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[4])<<std::endl;
    
    err = SendNRecv(this);

    if(FST_SUCCESS == err)
    {
       length = *(int *)&client_service_response_.res_buff[4];
       memcpy((char *)data,&client_service_response_.res_buff[8],length);      
    }
    else
    {
       length = 0;
    }    
    return err;
}


ERROR_CODE_TYPE fst_controller::ServoService::ReadIntVar(int size_of_varlist,const char *varname,int* res)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x40;
    unsigned int id = 0x20;
    short size = (short)size_of_varlist;
    memcpy(&client_service_request_.req_buff[0],(char *)&id,sizeof(id));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request_.req_buff[4],(char *)&size,sizeof(size));
    //std::cout<<*((short *)&client_service_request_.req_buff[4])<<std::endl;
    //std::cout<<*((short *)&client_service_request_.req_buff[5])<<std::endl;
    
    memcpy(&client_service_request_.req_buff[6],(char *)varname,1024-7);
    //std::cout<<&client_service_request_.req_buff[6]<<std::endl;
    
    err = SendNRecv(this);
    if(FST_SUCCESS == err)
    {
       memcpy((char *)res,&client_service_response_.res_buff[0],4*(size_of_varlist+1)); 
       if(1!=*(int*)&client_service_response_.res_buff[0])
           printf("Read Int response flag:%d\n", *(int*)&client_service_response_.res_buff[0]);
    } 
    else
    {
       printf("Read Int response fail\n");
    }     
    return err;
}

ERROR_CODE_TYPE fst_controller::ServoService::SetTrig(const char *trigname,unsigned short ticks)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x40;
    unsigned int id = 0x10;
    memcpy(&client_service_request_.req_buff[0],(char *)&id,sizeof(id));
    //std::cout<<*((unsigned int *)&client_service_request.req_buff[0])<<std::endl;
    memcpy(&client_service_request_.req_buff[4],(char *)&ticks,sizeof(ticks));
    std::cout<<*((short *)&client_service_request_.req_buff[4])<<std::endl;
    
    int len = strlen(trigname);
    memcpy(&client_service_request_.req_buff[6],trigname,len+1);
    
    err = SendNRecv(this);     
    return err;
}

ERROR_CODE_TYPE fst_controller::ServoService::ServoCMD(unsigned int id,const char * req,int req_size,char* res,int res_size)
{
    ERROR_CODE_TYPE err;
    client_service_request_.req_id = 0x60;
    memcpy(&client_service_request_.req_buff[0],(char *)&id,sizeof(id));
    
    memcpy(&client_service_request_.req_buff[4],req,req_size);
   
    err = SendNRecv(this);     
    if(FST_SUCCESS == err)
    {
       memcpy(res,&client_service_response_.res_buff[4],res_size); 
    } 
    else
    {
       printf("Servo CMD response fail\n");
    } 
    return err;
}


