/**
 * @file reg_interface.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-06-12
 */
#ifndef TP_INTERFACE_REG_INTERFACE_H_
#define TP_INTERFACE_REG_INTERFACE_H_

#include <atomic>
#include <string>
#include <pb_encode.h>
#include "motionSL.pb.h"
#include "base_types.pb.h"
#include "common.h"
#include "interpreter_common.h"
#include "error_code.h"

#include <vector>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <parameter_manager/parameter_manager_param_group.h>


struct RegTypeInfo
{
    std::string path;
    RegType type;
};

class RegInterface
{
  public:	 
    RegInterface();
    ~RegInterface();
    static RegInterface* instance();
    U64 initial();
    
    /**
     * @brief: set Reg 
     *
     * @param path: path of set Reg
     * @param value: value to set
     *
     * @return: 0 if success 
     */
    U64 setReg(RegMap* reg_info);


    /**
     * @brief:get Reg
     *
     * @param path: path to get
     * @param buffer: buffer to store value 
     * @param buf_len: buffer len
     * @param io_bytes_len: actual length
     *
     * @return: 0 if success 
     */
    U64 getReg(RegMap* reg_info, unsigned char *buffer, int buf_len, int& io_bytes_len);

    /**
     * @brief: check valid of this DI or DO 
     *
     * @param path: path of this DI or DO
     * @param io_info: io infomation 
     *
     * @return: true if it is valid 
     */
    U64 checkReg(const char *path, RegMap* reg_info);


  private:
    std::atomic<RegTypeInfo*>  reg_info_;  //
    std::atomic_int     io_num_;    //number of IO board
};

#endif
