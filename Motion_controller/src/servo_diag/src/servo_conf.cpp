#include <string.h>
#include <servo_conf.h>
#include <fstream>

fst_controller::Servconf::Servconf(const std::string &filename)
{
    bool succ = false;

    filename_ = filename;
    std::vector<int> stored_conf;
    p_paramgroup_ = new fst_parameter::ParamGroup(filename_);
        
    if(NULL != p_paramgroup_)
    {
        p_paramgroup_->getParam("servo.stored_start", startaddr_stored_);
        p_paramgroup_->getParam("servo.param_length", paramlength_);
        p_paramgroup_->getParam("servo.stored_length", stored_length_);
        p_paramgroup_->getParam("servo.stored_param", stored_conf);
        datastr_ = new char[paramlength_];
        
        if((NULL != datastr_)&&\
            (startaddr_stored_+stored_length_<=paramlength_)&&\
            ((int)stored_conf.size()*4==stored_length_))
        {
            
            for(int i = 0;i<stored_length_/4;i++)
            {
            	*(int *)&datastr_[i*4+startaddr_stored_] = stored_conf[i];
            }
            succ = true;

        }
    }
   
	if(false == succ)
	{
		printf("failed to initialize servo configuration");
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
        if((int)addr<paramlength_)
        {
            l_len = length;
            if((int)addr+l_len>paramlength_) 
                l_len = paramlength_ - (int)addr;
            memcpy(data,&datastr_[addr],l_len);
        }
    }    
    return l_len;    
}

void fst_controller::Servconf::saveConf(void)
{
    if ((NULL != datastr_)&&(NULL != p_paramgroup_))
    {
        std::vector<int> stored_conf;
        stored_conf.resize(stored_length_/4);
        if(startaddr_stored_+stored_length_<=paramlength_)
        {
            for(int i = 0;i<stored_length_/4;i++)
            {
            	stored_conf[i] = *(int *)&datastr_[i*4+startaddr_stored_];
            }
        }
        p_paramgroup_->setParam("servo.stored_param", stored_conf);
        p_paramgroup_->dumpParamFile(filename_);
    }
}


void fst_controller::Servconf::downloadConf(fst_controller::ServoService &serv,unsigned int 
addr,int length)
{
    char *data = new char[length];
    int l_len = getConf(addr, data,fst_controller::ServoService::SERVO_CONF_SEG);
    if(l_len>0)
    {
        serv.downloadParam(addr,data,l_len);
    }
    delete [] data;
}


int fst_controller::Servconf::uploadConf(fst_controller::ServoService &serv,int addr,int 
length)
{
    char *data = new char[length];
    ERROR_CODE_TYPE err = serv.uploadParam(addr,data,length);
    if(FST_SUCCESS==err)
    {
       setConf(addr,data,length);
       //std::cout<<*((char *)data)<<std::endl;
    }
    else
    {
        length  = 0;
    }
    //sleep(1);
    delete [] data;
    return length;

}

void fst_controller::Servconf::initDownloadConf(fst_controller::ServoService &serv)
{
    int l_len;
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    for(int addr = startaddr_stored_;addr<startaddr_stored_+stored_length_;)
    {
        l_len = startaddr_stored_+stored_length_-addr;
        l_len = (l_len>max_seg_len)?max_seg_len:l_len;

        downloadConf(serv, addr,l_len);
        addr += l_len;
    }	
}

void fst_controller::Servconf::initConfFile(fst_controller::ServoService &serv)
{
    int l_len;
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    for(int addr = startaddr_stored_;addr<startaddr_stored_+stored_length_;)
    {
        l_len = startaddr_stored_+stored_length_-addr;
        l_len = (l_len>max_seg_len)?max_seg_len:l_len;

        uploadConf(serv, addr,l_len);
        addr += l_len;
    }	
    saveConf();
}



