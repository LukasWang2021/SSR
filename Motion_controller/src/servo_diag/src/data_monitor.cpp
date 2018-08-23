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
    p_comm_ = new fst_comm_interface::CommInterface;
    start_monitor_ = false;
    record_fifo_ = NULL;
    char ip_str[30];
    if(NULL!=p_comm_)
    {
        snprintf(ip_str,30,"%s:%d", ip_addr, port);
        std::cout<<ip_str<<std::endl;
        ErrorCode fd = p_comm_->createChannel(COMM_REP, COMM_TCP, ip_str);
        openMem(MEM_CORE);
        if (fd == CREATE_CHANNEL_FAIL)
        {
            printf("Error when server createChannel.\n");
            delete p_comm_;
            p_comm_ = NULL;
        }
    }
}


fst_controller::DataMonitor::~DataMonitor(void)
{
    if(NULL!=p_comm_)
        delete p_comm_;
}

void fst_controller::DataMonitor::initDataMonitor(void)
{
    if (NULL == record_fifo_)
        record_fifo_ = new fst_controller::LimitedFifo<Servo_Data_Record_t>(MAX_SNAPSHOT_SIZE);
    if (NULL!=record_fifo_)
    {
        boost::thread thrd_moni(boost::bind(pcComm_Thread, this));
        snapshot_size_ = MAX_SNAPSHOT_SIZE/2; //default snapshot size
        thrd_moni.detach();
        printf("PC monitor start!\n");
    }
    else
    {
        printf("Can not allocate memory for record fifo!\n");
    }
}

void fst_controller::DataMonitor::startMonitor(DataMonitor* moni,
                                                std::vector<int>& t_list,
                                                unsigned int &ss_size)
{   
    if(false == moni->start_monitor_)
    {
        Servo_Data_Record_t record;
        moni->record_fifo_->lock_push();
        while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
        moni->t_list_ = t_list;
        moni->start_monitor_ = true;
        moni->data_state_ = -1;
        moni->record_fifo_->unlock_push();              
    }
    ss_size = moni->snapshot_size_;
}

void fst_controller::DataMonitor::setSnapshotSize(DataMonitor* moni,unsigned int ss_size)
{   
    if(false == moni->start_monitor_)
    {
        if(ss_size > MAX_SNAPSHOT_SIZE)
            ss_size = MAX_SNAPSHOT_SIZE;
        moni->snapshot_size_ = ss_size;            
    }
}

void fst_controller::DataMonitor::stopMonitor(DataMonitor* moni)
{
    moni->start_monitor_ = false;    
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

bool fst_controller::DataMonitor::sendResponse(fst_comm_interface::CommInterface* pcomm,const 
void *buf, int buf_size)
{
    ErrorCode rc;
    bool res = false;
    int retry = 6;
    do
    {
        rc = pcomm->send(buf, 
                buf_size, 
                COMM_DONTWAIT);
        retry--;
    }while(SUCCESS != rc && retry>0);  

    if(SUCCESS == rc)
        res = true;
    return res;
}


bool fst_controller::DataMonitor::onRecordRequest(DataMonitor* moni)
{
    bool res = false;
    
    unsigned char resp;
    if(moni->data_state_>0)
    {
        resp = 0xA1;
        if(true == sendResponse(moni->p_comm_,&resp,sizeof(resp)))
        {
            res = true;
        }
        // according to nanomsg request response communication mechanizm, if there was 
        // no respose within one minutes, the sender will repeat sending
        //Servo_Data_Record_t record;
        //while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
        //moni->record_fifo_->unlock_push();
    }
    else 
    {
        resp = 0xA0;
        sendResponse(moni->p_comm_,&resp,sizeof(resp));
        if(moni->data_state_<0)
        {
            moni->data_state_ = 0;
        }
    }
    
    return res;
}

void fst_controller::DataMonitor::onFinishRecord(DataMonitor* moni)
{
    Servo_Data_Record_t record;
    while(moni->record_fifo_->fetch_item(record)>0);// clear queue  , !!>0
    moni->data_state_ = -1;
    moni->record_fifo_->unlock_push();
    //printf("Push unlocked\n");
}



int fst_controller::DataMonitor::onGetdataRequest(unsigned char seq,DataMonitor* moni)
{
    int res;
    static int l_seq = -1;
    static int cnt = 0;
    int cnt_help = 0;
    int reqst_seq = int(seq);
    if((0==seq)&&(moni->record_fifo_->size()==(int)(moni->snapshot_size_)))
    {
        l_seq = -1;
        cnt = 0;
        //printf("Prepared to send fist package\n");
    }
    if(moni->data_state_>0)
    {
        //data has prepared
        
        if(reqst_seq == l_seq + 1)
        {
            //right sequence
            moni->data_package_.length = moni->record_fifo_->size();
            if(moni->data_package_.length>PACKAGE_SIZE) 
                moni->data_package_.length = PACKAGE_SIZE;

            moni->data_package_.length = moni->record_fifo_->fetch_batch(
                                                           moni->data_package_.record,
                                                           moni->data_package_.length);
            moni->data_package_.seq = reqst_seq;
            cnt_help = moni->data_package_.length;
            //printf("Get %d records from fifo, according to %dth sequence\n",cnt_help,reqst_seq);
        }
        else if(reqst_seq != l_seq)
        {
            //printf("Wrong sequence number!!!\n");
        }
        else
        {
            //retry
            //do nothing
            cnt_help = 0;
        }

        res = sendResponse(moni->p_comm_,
                         &moni->data_package_, 
                         sizeof(moni->data_package_));

        if(true == res)
        {
            l_seq = moni->data_package_.seq;
            cnt += cnt_help;
            //printf("%d records has been sent\n",cnt);
        }


        
        if((moni->data_package_.length<=0)||(true != res))
        {
            onFinishRecord(moni);
        }

    }
    else
    {
        moni->data_package_.length = 0;
        moni->data_package_.seq = -1;
        l_seq = moni->data_package_.seq;
        moni->p_comm_->send(&moni->data_package_, 
                    sizeof(moni->data_package_), 
                    COMM_DONTWAIT);
        printf("ERR: get data before data is ready!!!\n");
    }    
    return l_seq;
}

void fst_controller::DataMonitor::pcComm_Thread(DataMonitor* moni)
{
    if(NULL!=moni->p_comm_)
    {
        boost::thread thrd_data_moni(boost::bind(dataMonitor_Thread, moni));
        //send service to core1
        thrd_data_moni.detach();

        
        unsigned char req[2];
        while(1)
        {
            if(false == moni->start_monitor_)
            {
                usleep(1000);
                continue;
            }
            if(moni->record_fifo_->size()>=(int)(moni->snapshot_size_))
            {
                if(0 == moni->data_state_)
                {
                    moni->record_fifo_->lock_push();    
                    moni->data_state_ = 1;
                    //printf("data triggered and locked\n");
                }
            }
            if(moni->p_comm_->recv(&req, 2, COMM_DONTWAIT)!=SUCCESS)
            {
                usleep(1000);

                continue;
            }
            switch(req[0])
            {
                case 0xC1:   
                {
                    onRecordRequest(moni);
                    break;
                }
                case 0xC2:
                {
                    onGetdataRequest(req[1],moni);
                    break;
                }
        
                default:
                    break;
                
            }
        }
    }    
}
void fst_controller::DataMonitor::dataMonitor_Thread(DataMonitor* moni)
{
    Servo_Data_Record_t servo_record;
    LOG_RECORD_T record;
    while(1)
    {
        while(getRecord(&record)>=0)
        {
            int k = 0;
            
            if(false == moni->start_monitor_)
            {
                usleep(10);
                continue;
            }
            for (int i = 0;i<record.length&&i<(int)moni->t_list_.size();i++)
            {
                k = logdata2Databuf(servo_record.data,k,&(record.data[i]),moni->t_list_[i]);
            }
            servo_record.flag = (int)record.time_flag;
            if(0==servo_record.flag)
            {
                moni->record_fifo_->push_item(servo_record,true,moni->snapshot_size_/2);//push anyway
            }
            else
            {
                moni->record_fifo_->push_item(servo_record,true,moni->snapshot_size_);//push anyway
            }
        }
        if(false == moni->start_monitor_)
        {
            usleep(100);
        }
        usleep(10);//delay 50ms is allowed
    }
}


int fst_controller::DataMonitor::logdata2Databuf(char *databuf,
                                                    int pos,const data64b_t* record_data,
                                                    int type)
{
    switch(type)
    {
        case TYPE_INT8:
        {
            if(alignNcheck(pos,1,RECORD_SIZE))
            {
                *(char *)&databuf[pos] = *(char *)record_data;
                pos += 1;
            }
            break;
        }  
        case TYPE_UINT8:
        {
            if(alignNcheck(pos,1,RECORD_SIZE))
            {
                *(unsigned char *)&databuf[pos] = *(unsigned char *)record_data;
                pos += 1;
            }
            break;
        }
        case TYPE_INT16:
        {
            if(alignNcheck(pos,2,RECORD_SIZE))
            {
                *(short *)&databuf[pos] = *(short *)record_data;
                pos += 2;
            }
            break;
        }    
        case TYPE_UINT16:
        {
            if(alignNcheck(pos,2,RECORD_SIZE))
            {
                *(unsigned short *)&databuf[pos] = *(unsigned short *)record_data;
                pos += 2;
            }
            break;
        } 

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
        case TYPE_INT64:
        {
            if(alignNcheck(pos,8,RECORD_SIZE))
            {
                *(int64_t *)&databuf[pos] = *(int64_t *)record_data;
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



