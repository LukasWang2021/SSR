#include <string.h>
#include <data_monitor.h>
#include <fstream>
#include <iostream>
#include "logfifo.h"
#include <boost/thread.hpp>
#include "middleware_to_mem/middleware_to_sharedmem.h"

fst_controller::DataMonitor::DataMonitor(const char *ip_addr, int port)
{
    //create communication channel.
    p_comm = new fst_comm_interface::CommInterface;
    start_monitor = false;
    record_fifo_ = NULL;
    char ip_str[30];
    if(NULL!=p_comm)
    {
        snprintf(ip_str,30,"%s:%d", ip_addr, port);
        std::cout<<ip_str<<std::endl;
        ERROR_CODE_TYPE fd = p_comm->createChannel(COMM_REP, COMM_TCP, ip_str);
        openMem(MEM_CORE);
        if (fd == CREATE_CHANNEL_FAIL)
        {
            printf("Error when server createChannel.\n");
            delete p_comm;
            p_comm = NULL;
        }
    }
}


fst_controller::DataMonitor::~DataMonitor(void)
{
    if(NULL!=p_comm)
        delete p_comm;
}

void fst_controller::DataMonitor::initDataMonitor(void)
{
    if (NULL == record_fifo_)
        record_fifo_ = new fst_controller::LimitedFifo<Servo_Data_Record_t>(SNAPSHOT_SIZE);
    if (NULL!=record_fifo_)
    {
        boost::thread thrd_moni(boost::bind(pcComm_Thread, this));
        thrd_moni.detach();
        printf("PC monitor start!\n");
    }
    else
    {
        printf("Can not allocate memory for record fifo!\n");
    }
}

void fst_controller::DataMonitor::StartMonitor(DataMonitor* moni,std::vector<int>& t_list)
{   
    if(false == moni->start_monitor)
    {
        Servo_Data_Record_t record;
        moni->record_fifo_->lock_push();
        while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
        moni->t_list_ = t_list;
        moni->start_monitor = true;
        moni->record_fifo_->unlock_push();                
    }
}

void fst_controller::DataMonitor::StopMonitor(DataMonitor* moni)
{
    moni->start_monitor = false;    
}

bool fst_controller::DataMonitor::alignNcheck(int & k,int bytesize,int totalsize)
{
    int l = k%bytesize;
    if(l>0)
    {
        k = k-l+bytesize;
    }
    if(k+bytesize>totalsize)
    {
        return false;
    }
    return true;
}

void fst_controller::DataMonitor::pcComm_Thread(DataMonitor* moni)
{
    if(NULL!=moni->p_comm)
    {
        boost::thread thrd_data_moni(boost::bind(DataMonitor_Thread, moni));
        //send service to core1
        thrd_data_moni.detach();

        
        unsigned char req;
        while(1)
        {
            if(false == moni->start_monitor)
            {
                sleep(0);
                continue;
            }

            if(moni->p_comm->recv(&req, sizeof(req), COMM_DONTWAIT)!=FST_SUCCESS)
            {
                usleep(1000);

                continue;
            }
            switch(req)
            {
                case 0xC1:
                {
                    //int cnt = 0;  
                    printf("C1 received\n");
                    if(moni->record_fifo_->is_push_locked())
                    {
                        Servo_Data_Record_t record;
                        while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
                        moni->record_fifo_->unlock_push();
                    }

                    while((true == moni->start_monitor)&&\
                         (moni->record_fifo_->size()<SNAPSHOT_SIZE))
                    {
                       //printf("fifo length: %d\n", moni->record_fifo_->size());
                       usleep(10000); 
                    }
                    moni->data_package.length = moni->record_fifo_->size();
                    if(moni->data_package.length>PACKAGE_SIZE) 
                        moni->data_package.length = PACKAGE_SIZE;
                    moni->record_fifo_->lock_push();
                    moni->record_fifo_->fetch_batch(moni->data_package.record,moni->data_package.length);
                    moni->p_comm->send(&moni->data_package, 
                                    sizeof(moni->data_package), 
                                    COMM_DONTWAIT);
                    break;
                }    
                case 0xC2:
                {
                    moni->data_package.length = moni->record_fifo_->size();
                    if(moni->data_package.length>PACKAGE_SIZE) 
                        moni->data_package.length = PACKAGE_SIZE;
                    moni->record_fifo_->fetch_batch(moni->data_package.record,moni->data_package.length);
                    moni->p_comm->send(&moni->data_package, 
                                    sizeof(moni->data_package), 
                                    COMM_DONTWAIT);
                    if(moni->data_package.length<=PACKAGE_SIZE) 
                    {
                        Servo_Data_Record_t record;
                        while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
                        moni->record_fifo_->unlock_push();
                    }
                    //printf("second package length: %d\n", moni->data_package.length);
                    break;
                }           
                default:
                    break;
                
            }
        }
    }    
}
void fst_controller::DataMonitor::DataMonitor_Thread(DataMonitor* moni)
{
    Servo_Data_Record_t servo_record;
    LOG_RECORD_T record;
    int n = 0;
    while(1)
    {
        while(getRecord(&record)>=0)
        {
            int k = 0;
            
            if(false == moni->start_monitor)
            {
                sleep(0);
                continue;
            }
            for (int i = 0;i<record.length&&i<moni->t_list_.size();i++)
            {
                k = Logdata2Databuf(servo_record.data,k,&(record.data[i]),moni->t_list_[i]);
            }
            servo_record.flag = (int)record.time_flag;

            moni->record_fifo_->push_item(servo_record,true);//push anyway
        }
        //usleep(1000);
    }
}


int fst_controller::DataMonitor::Logdata2Databuf(char *databuf,
                                                    int pos,const data64b_t* record_data,
                                                    int type)
{
    switch(type)
    {
        case TYPE_INT32:
        {
            if(alignNcheck(pos,4,RECORD_SIZE))
            {
                *(int *)&databuf[pos] = *(int *)record_data;
                pos += 4;
            }
            break;
        }
        case TYPE_UINT32:
        {
            if(alignNcheck(pos,4,RECORD_SIZE))
            {
                *(unsigned int *)&databuf[pos] = *(unsigned int *)record_data;
                pos += 4;
            }
            break;
        }  
        case TYPE_FLOAT32:
        {
            if(alignNcheck(pos,4,RECORD_SIZE))
            {
                *(float *)&databuf[pos] = *(float *)record_data;
                pos += 4;
            }
            break;
        } 
        case TYPE_FLOAT64:
        {
            if(alignNcheck(pos,8,RECORD_SIZE))
            {
                *(double *)&databuf[pos] = *(double *)record_data;
                pos += 8;
            }
            break;
        }    
        default:
        {
            break;
        }
    }	
    return pos;
}

/*
TYPE_UINT8 = 0,
TYPE_INT8 = 1,
TYPE_UINT16 = 2,
TYPE_INT16 = 3,
TYPE_UINT32 = 4,
TYPE_INT32 = 5,
TYPE_FLOAT32 = 6,
TYPE_FLOAT64 = 7,
*/


