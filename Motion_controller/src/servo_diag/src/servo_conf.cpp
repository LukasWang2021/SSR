#include <string.h>
#include <servo_conf.h>
#include <fstream>
#include <iostream>

fst_controller::Servconf::Servconf(const std::string &filename)
{
    bool succ = false;

    filename_ = filename;
    std::vector<int> stored_conf;
    p_paramgroup_ = new fst_parameter::ParamGroup(filename_);
        
    if (NULL != p_paramgroup_)
    {

        p_paramgroup_->getParam("servo.stored_start", startaddr_stored_);
        p_paramgroup_->getParam("servo.param_length", paramlength_);
        p_paramgroup_->getParam("servo.stored_length", stored_length_);
        p_paramgroup_->getParam("servo.stored_param", stored_conf);

        datastr_ = new char[paramlength_];

        std::cout << "stored_start=" << startaddr_stored_ << std::endl;
        std::cout << "stored_length=" << stored_length_ << std::endl;
        std::cout << "param_length" << paramlength_ << std::endl;
        std::cout << "sizeof stored param" << stored_conf.size() << std::endl;
        
        if ((NULL != datastr_) &&\
            (startaddr_stored_ + stored_length_ <= paramlength_) &&\
            ((int)stored_conf.size() * 4 == stored_length_))
        {
            for (int i = 0; i < stored_length_ / 4; i++)
            {
            	*(int *)&datastr_[i * 4 + startaddr_stored_] = stored_conf[i];
            }
            succ = true;
        }
    }
   
	if (false == succ)
	{
        std::cout << "failed to initialize servo configuration" << std::endl;
	}
    else
    {
        std::cout << "initialize servo configuration success" << std::endl;
    }
}

fst_controller::Servconf::~Servconf(void)
{
    if(NULL!=datastr_)
    {
        delete [] datastr_;
    }
	if(NULL != p_paramgroup_)
	{
		delete p_paramgroup_;
	}
}

int fst_controller::Servconf::setConf(unsigned int addr, const char *data, int length)
{
    int l_len = 0;
    if (NULL != datastr_)
    {
        if((int)addr<paramlength_)
        {
            l_len = length;
            if((int)addr+l_len>paramlength_) 
                l_len = paramlength_ - (int)addr;
            memcpy(&datastr_[addr],data,l_len);
        }
    }    
    return l_len;
}

int fst_controller::Servconf::getConf(unsigned int addr, char *data, int length)
{
    int l_len = 0;

    if (NULL != datastr_)
    {
        if ((int)addr < paramlength_)
        {
            l_len = length;
            if ((int)addr + l_len > paramlength_) 
                l_len = paramlength_ - (int)addr;
            memcpy(data, &datastr_[addr], l_len);
        }
    }

    return l_len;    
}

void fst_controller::Servconf::saveConf(void)
{
    std::cout << "saveConf: datastr=" << datastr_ << std::endl;
    if ((NULL != datastr_) && (NULL != p_paramgroup_))
    {
        std::vector<int> stored_conf;
        stored_conf.resize(stored_length_ / 4);

        if (startaddr_stored_ + stored_length_ <= paramlength_)
        {
            for (int i = 0; i < stored_length_ / 4; i++)
            {
            	stored_conf[i] = *(int *)&datastr_[i * 4 + startaddr_stored_];
            }
        }

        bool res = true;
        res = res && p_paramgroup_->setParam("servo.stored_param", stored_conf);
        res = res && p_paramgroup_->dumpParamFile(filename_);

        std::cout << "saveConf: " << res << std::endl;
    }
}


int fst_controller::Servconf::downloadConf(fst_controller::ServoService &serv, 
                                           int startaddr,int length)
{
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    char *data;
    int l_len;

    if (startaddr < 0 || length < 0)
    {
        std::cout << "fail and exit" << std::endl;
        return 0;
    }
    
    data = new char[length];
    
    int addr;
    ErrorCode err;
    l_len = getConf(startaddr, data, length);

    if (l_len > 0)
    {
        for (addr = startaddr; addr < startaddr + l_len;)
        {
            int seg_len;//calculate seg length
            seg_len = startaddr + l_len - addr;
            seg_len = (seg_len > max_seg_len) ? max_seg_len : seg_len;
            
            err = serv.downloadParam(addr, &data[addr - startaddr], seg_len);
            
            if (SUCCESS == err)
            {
                std::cout << "downloadParam done" << std::endl;
                std::cout << "addr=" << addr << ", startaddr=" << startaddr << ", seg_len=" << seg_len << std::endl;
                addr += seg_len;
            }
            else
            {
                std::cout << "downloadParam failed" << std::endl;
                std::cout << "addr=" << addr << ", startaddr=" << startaddr << ", seg_len=" << seg_len << std::endl;
                break;
            }
        }  
        //calculate how many bytes has been sent
        l_len = addr - startaddr;
    }

    delete [] data;
    return l_len;
}


int fst_controller::Servconf::uploadConf(fst_controller::ServoService &serv, int startaddr, int length)
{
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    char *data;
    int l_len = length;

    std::cout << "uploadConf: startaddr=" << startaddr << ", length=" << length << std::endl;
    if (startaddr < 0 || length < 0)
    {
        std::cout << "fail and exit" << std::endl;
        return 0;
    }
    
    data = new char[l_len];

    int addr;
    ErrorCode err;

    if (l_len > 0)
    {
        for (addr = startaddr; addr < startaddr + l_len;)
        {
            int seg_len;//calculate seg length
            seg_len = startaddr + l_len - addr;
            seg_len = (seg_len > max_seg_len) ? max_seg_len : seg_len;
            
            std::cout << "upload param: addr=" << addr << ", startaddr=" << startaddr << ", seg_len=" << seg_len;
            err = serv.uploadParam(addr, &data[addr - startaddr], seg_len);
            if (SUCCESS == err)
            {
                std::cout << " - done" << std::endl;
                addr += seg_len;
            }
            else
            {
                std::cout << " - failed" << std::endl;
                break;
            }
        }  
        //calculate how many bytes has been received
        l_len = addr - startaddr;
    }
    
    if (SUCCESS == err)
    {
        l_len = setConf(startaddr, data, l_len);
        std::cout << "success" << std::endl;
       //std::cout<<*((char *)data)<<std::endl;
    }
    else
    {
        std::cout << "failed" << std::endl;
        l_len  = 0;
    }

    delete [] data;
    return l_len;
}


void fst_controller::Servconf::initDownloadConf(fst_controller::ServoService &serv)
{	
    downloadConf(serv, startaddr_stored_, stored_length_);
}

void fst_controller::Servconf::initConfFile(fst_controller::ServoService &serv)
{
    uploadConf(serv, startaddr_stored_, stored_length_);
    saveConf();
}




