/**
 * @file io_interface.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-06-12
 */

#include "io_interface.h"
#include "common.h"
#include "fst_error.h"
#include <boost/algorithm/string.hpp>

IOInterface::IOInterface()
{
    
}

IOInterface::~IOInterface()
{
   if (io_manager_ != NULL)
       delete io_manager_;
   if (dev_info_ != NULL)
       delete [] dev_info_;
}

U64 IOInterface::initial()
{
    U64 result = 0;
    io_manager_ = new fst_io_manager::IOManager;
    result = io_manager_->init(1);
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("io_manager_ init failed:%llx", result);
        return result;
    }
    //-----------------get num of devices.----------------------//
    io_num_ = io_manager_->getDevicesNum();
    FST_INFO("io_num_:%d",io_num_.load());

    dev_info_ = new fst_io_manager::IODeviceInfo[io_num_];

    for (int i = 0; i < io_num_; i++)
    {
        result = io_manager_->getDeviceInfo(i, dev_info_[i]);
        //FST_INFO("input:%d,output:%d", dev_info_[i].input, dev_info_[i].output);
        if (result != TPI_SUCCESS)
            return result;
    }

    return TPI_SUCCESS;
}


int IOInterface::getIODevNum()
{
    return io_num_;
}

fst_io_manager::IODeviceInfo* IOInterface::getDevInfoPtr()
{
    boost::mutex::scoped_lock lock(mutex_);
    return dev_info_;
}

U64 IOInterface::setDO(const char *path, char value)
{
    std::vector<std::string> vc_path;
    boost::split(vc_path, path, boost::is_any_of("/"));
    int size = vc_path.size();
    if ((size != 6) || (vc_path[0] != "root") 
    || (vc_path[1] != "IO") || (vc_path[4] != "DO"))
    {
        return INVALID_PATH_FROM_TP;        
    }
    fst_io_manager::IODeviceInfo *info = getDevInfoPtr();
    for (int i = 0; i < getIODevNum(); i++)
    {
        if ((vc_path[2] == info[i].communication_type) 
        && (stoi(vc_path[3]) == info[i].device_number))
        {
            FST_INFO("id:%d, port:%d, value:%d", info[i].id, stoi(vc_path[5]), value);
            return io_manager_->setModuleValue(info[i].id, stoi(vc_path[5]), value);        }
    }
    return PARSE_IO_PATH_FAILED;
}


U64 IOInterface::setDO(int msg_id, unsigned char value)
{
    int index = msg_id % IO_MAX_NUM;
    int dev_address = msg_id - index;
    FST_INFO("dev_address:%d,index:%d, value:%d", dev_address, index, value);
    return io_manager_->setModuleValue(dev_address, index, value);
}

U64 IOInterface::getDIO(const char *path, unsigned char *buffer, int buf_len, int& io_bytes_len)
{
    std::vector<std::string> vc_path;
    boost::split(vc_path, path, boost::is_any_of("/"));
    
    if ((vc_path[0] != "root") || (vc_path[1] != "IO"))
    {
        return INVALID_PATH_FROM_TP;        
    }
    int size = vc_path.size();
    fst_io_manager::IODeviceInfo *info = getDevInfoPtr();
    for (int i = 0; i < getIODevNum(); i++)
    {
        if ((vc_path[2] == info[i].communication_type) 
        && (stoi(vc_path[3]) == info[i].device_number))
        {
            if (size == 4)
            {
                int io_len;
                U64 result = io_manager_->getModuleValues(info[i].id, buf_len, buffer, io_bytes_len);
                io_bytes_len = ((info[i].input + 7) >> 3) + ((info[i].output + 7) >> 3);
                /*printf("==buffer:");*/
                //for(int j = 0; j < io_bytes_len; j++)
                    //printf("%xd ", buffer[j]);
                /*printf("\n");*/
                return result;
            }
            else if (size == 6)
            {
                io_bytes_len = 1;
                if (vc_path[4] == "DI")
                {
                    FST_INFO("get module value input:id:%d, index:%d",info[i].id, stoi(vc_path[5]));
                    return io_manager_->getModuleValue(info[i].id, IO_INPUT, stoi(vc_path[5]), buffer[0]);
                }
                else if (vc_path[4] == "DO")
                {
                    FST_INFO("get module value output:id:%d, index:%d", info[i].id, stoi(vc_path[5]));
                    return io_manager_->getModuleValue(info[i].id, IO_OUTPUT, stoi(vc_path[5]), buffer[0]);
                }
                else
                {
                    return PARSE_IO_PATH_FAILED;
                }
            }
            else 
            {
                return PARSE_IO_PATH_FAILED;
            }
        }
    }
    return PARSE_IO_PATH_FAILED;
}


U64 IOInterface::getDIO(int msg_id, uint8_t *buffer, int buf_len, int& io_bytes_len)
{
    unsigned int index = msg_id % IO_MAX_NUM;
    int dev_address = msg_id - index;
//FST_INFO("index:%d,dev_address:%d", index, dev_address);
    fst_io_manager::IODeviceInfo *dev = getIODevPtr(dev_address);
    if (index == 0)
    {
        int io_len;
        U64 result = io_manager_->getModuleValues(dev->id, buf_len, buffer, io_len);
        io_bytes_len = ((dev->input + 7) >> 3) + ((dev->output + 7) >> 3);
        /*printf("==++buffer:");*/
                //for(int j = 0; j < io_bytes_len; j++)
                    //printf("%x ", buffer[j]);
                /*printf("\n");*/

        return result;
    }
    else
    {
        io_bytes_len = 1;
        if (index <= dev->input)
        {
            return io_manager_->getModuleValue(dev->id, IO_INPUT, index, buffer[0]);
        }
        else
        {
            unsigned int new_index = index - dev->input;
            return io_manager_->getModuleValue(dev->id, IO_OUTPUT, new_index, buffer[0]);
        }
    }
}

U64 IOInterface::checkIO(const char *path, int& buf_len, uint32_t& msg_id)
{
    std::vector<std::string> vc_path;
    boost::split(vc_path, path, boost::is_any_of("/"));
    
    if ((vc_path[0] != "root") || (vc_path[1] != "IO"))
    {
        return INVALID_PATH_FROM_TP;        
    }
    int size = vc_path.size();
    fst_io_manager::IODeviceInfo *info = getDevInfoPtr();
    for (int i = 0; i < getIODevNum(); i++)
    {
        if ((vc_path[2] == info[i].communication_type) 
        && (stoi(vc_path[3]) == info[i].device_number))
        {
            msg_id = info[i].id;
            if (size == 4)
            {
                buf_len = ((info[i].input + 7) >> 3) + ((info[i].output + 7) >> 3);
                return TPI_SUCCESS;
            }
            else if (size == 6)
            {
                int index = stoi(vc_path[5]);
                if (vc_path[4] == "DI")
                {   
                    if (index > (int)info[i].input)
                        return INVALID_PATH_FROM_TP;
                    msg_id = msg_id + index;
                }
                else if (vc_path[4] == "DO")
                {   
                    if (index > (int)info[i].output)
                        return INVALID_PATH_FROM_TP;
                    msg_id = msg_id + info[i].input + index;
                }
                else
                {
                    return INVALID_PATH_FROM_TP;
                }
                buf_len =  1;
                return TPI_SUCCESS;
            }
            else 
            {
                return PARSE_IO_PATH_FAILED;
            }
        }
    }
    return PARSE_IO_PATH_FAILED;

}

U64 IOInterface::checkIO(int msg_id, int& buf_len)
{
    unsigned int index = msg_id % IO_MAX_NUM;
    int dev_address = msg_id - index;

    fst_io_manager::IODeviceInfo *dev = getIODevPtr(dev_address);
    if (index == 0)
    {
        buf_len = ((dev->input + 7) >> 3) + ((dev->output + 7) >> 3);
    }
    else
    {        
        if (index > (dev->input + dev->output))
        {            
            return INVALID_PARAM_FROM_TP;
        }
        buf_len =  1;
    }
    return TPI_SUCCESS;
}




fst_io_manager::IODeviceInfo* IOInterface::getIODevPtr(int dev_address)
{
    boost::mutex::scoped_lock lock(mutex_);
    for (int i = 0; i < io_num_; i++)
    {        
        if ((int)dev_info_[i].id == dev_address)
        {
            return &dev_info_[i];
        }
   }

   return NULL;
}


U64 IOInterface::getIOError()
{
    return io_manager_->getIOError();
}

