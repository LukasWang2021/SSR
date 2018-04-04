/**
 * @file tpif_data.h
 * @brief: data used for tp_interface sharing with controller
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-30
 */

#ifndef TPIF_DATA_H_
#define TPIF_DATA_H_

#include <atomic>
#include "motionSL.pb.h"


typedef enum _TPCmdType
{
    SET,
    GET,
    CMD,
    OVERWRITE,
}TPCmdType;

typedef enum _TPRepType
{
    STATUS, 
    PARAM,
    LIST,
}TPRepType;


template <typename T = uint8_t,  int ParamBufLen = 1024*10>
class TPIFData
{
  public:
    TPIFData()
    {
        filled_ = false;
        buf_len_ = ParamBufLen * sizeof(T);
    }
    ~TPIFData(){}

    virtual void fillData(const char* buffer, int len)
    {
        memset(param_buffer_, 0, ParamBufLen);
        memcpy(param_buffer_, buffer, len);
        param_len_ = len;
    }

    virtual bool fillDataTimeout(const char* buffer, int len, int timeout=100000)
    {
        if (waitEmptyTimeout(timeout) == false)
        {
            return false;    
        }

        memset(param_buffer_, 0, ParamBufLen);
        memcpy(param_buffer_, buffer, len);
        param_len_ = len;

        return true;
    }

    virtual bool isFilled()
    {
        return filled_.load();
    }
     
    virtual bool waitEmptyTimeout(int timeout)
    {
       int interval = 100;
        int count = timeout / interval;
        while (isFilled())
        {
            usleep (interval);
            if (count-- <= 0)
            {
                FST_ERROR("fill data timeout");
                return false;
            }
        }
        return true; 
    }

    virtual int getParamBufLen()
    {
        return buf_len_;
    }

    virtual int getParamLen()
    {
        return param_len_;
    }

    virtual void setParamLen(int len)
    {
        param_len_ = len;
    }

    virtual void setFilledFlag(bool flag)
    {
        filled_.store(flag);
    }

    virtual T* getParamBufPtr()
    {
        return param_buffer_;
    }
  private:
    T param_buffer_[ParamBufLen];
    int buf_len_;
    int param_len_;
    std::atomic<bool> filled_;    
};

template <int PathBufLen = 128, typename T = char>
class TPIFReqData:public TPIFData<>
{
  public:    
    TPIFReqData()
    {
        
    }
    ~TPIFReqData(){}

    bool update(TPCmdType cmd_type, const char* path = NULL, int id = 0, int timeout=100000)
    {
        if (waitEmptyTimeout(timeout) == false)
        {
            return false;    
        }
        setFilledFlag(true);
        setType(cmd_type);
        setID(id);
        if (path != NULL)
        {
            memset(path_buffer_, 0, PathBufLen);
            path_len_ = strlen(path);
            memcpy(path_buffer_, path, strlen(path));
        }
        return true;
    }

    int getPathBufLen()
    {
        return PathBufLen;
    }
    int getPathLen()
    {
        return path_len_;
    }
    void setPathLen(int len)
    {   
        path_len_ = len;
    }
    int getType()
    {
        return type_;
    }
    void setType(TPCmdType type)
    {
        type_ = type;
    }
    T* getPathBufPtr()
    {
        return path_buffer_;
    }
    void setID(int id)
    {
        id_ = id;
    }
    int getID()
    {
        return id_;
    }

  private:
    T path_buffer_[PathBufLen];
    int         path_len_;
    int         id_;
    TPCmdType   type_;    
};

class TPIFRepData:public TPIFData<>
{
  public:   
    TPIFRepData(){}
    ~TPIFRepData(){}

    TPRepType getType()
    {
        return type_;
    }
    void setType(TPRepType type)
    {
        type_ = type;
    }

    void setID(int id)
    {
        id_ = id;
    }
    int getID()
    {
        return id_;
    }
  private:
    TPRepType   type_;
    int         id_;
};

typedef TPIFData<motion_spec_SignalGroup, 1> TPIPubData;

#endif //#ifndef TPIF_DATA_H_
