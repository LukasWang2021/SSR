/**
 * @file io_interface.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-06-12
 */
#ifndef TP_INTERFACE_IO_INTERFACE_H_
#define TP_INTERFACE_IO_INTERFACE_H_

#include <string>
#include <atomic>
#include <boost/filesystem.hpp>
#include "io_manager/io_manager.h"


#define IO_BASE_ADDRESS (100000)
#define IO_MAX_NUM      (1000)


class IOInterface
{
  public:	 
    IOInterface();
    ~IOInterface();
    U64 initial();
    /**
     * @brief: add io device infomation to dev_info_ 
     *
     * @return: 0 if success 
     */
    U64 addIODevices();

    /**
     * @brief: get io devices number 
     *
     * @return: 
     */
    int getIODevNum();
    
    /**
     * @brief 
     *
     * @return 
     */
    fst_io_manager::IODeviceInfo *getDevInfoPtr();

    U64 setDO(const char *path, char value);
    U64 setDO(int msg_id, unsigned char value);

    U64 getDIO(const char *path, unsigned char *buffer, int buf_len, int& io_bytes_len);
    U64 getDIO(int msg_id, uint8_t *buffer, int buf_len, int& io_bytes_len);

    U64 checkIO(const char *path, int& buf_len, uint32_t& msg_id);
    U64 checkIO(int msg_id, int& buf_len);

    fst_io_manager::IODeviceInfo* getIODevPtr(int dev_address);
    U64 getIOError();

  private:
    fst_io_manager::IOManager *io_manager_;
    std::atomic<int> io_num_;
    fst_io_manager::IODeviceInfo *dev_info_;

    boost::mutex	mutex_;		//mutex lock
};

#endif
