#include <string.h>
#include <servo_conf.h>
#include <fstream>

fst_controller::Servconf::Servconf(const std::string &filename)
{
    bool succ = false;

    m_filename = filename;
    std::vector<int> stored_conf;
    p_paramgroup_ = new fst_parameter::ParamGroup(m_filename);
        
    if(NULL != p_paramgroup_)
    {
        p_paramgroup_->getParam("servo.stored_start", m_startaddr_stored);
        p_paramgroup_->getParam("servo.param_length", m_length);
        p_paramgroup_->getParam("servo.stored_length", m_stored_length);
        p_paramgroup_->getParam("servo.stored_param", stored_conf);
        m_datastr = new char[m_length];
        
        if((NULL != m_datastr)&&\
            (m_startaddr_stored+m_stored_length<=m_length)&&\
            (stored_conf.size()*4==m_stored_length))
        {
            
            for(int i = 0;i<m_stored_length/4;i++)
            {
            	*(int *)&m_datastr[i*4+m_startaddr_stored] = stored_conf[i];
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
    if(NULL!=m_datastr)
    {
        delete [] m_datastr;
    }
	if(NULL != p_paramgroup_)
	{
		delete p_paramgroup_;
	}
}

int fst_controller::Servconf::Setconf(unsigned int addr, const char *data, int length)
{
    int l_len = 0;
    if (NULL != m_datastr)
    {
        if(addr<m_length)
        {
            l_len = length;
            if(addr+l_len>m_length) 
                l_len = m_length - addr;
            memcpy(&m_datastr[addr],data,l_len);
        }
    }    
    return l_len;
}

int fst_controller::Servconf::Getconf(unsigned int addr, char *data, int length)
{
    int l_len = 0;    
    if (NULL != m_datastr)
    {
        if(addr<m_length)
        {
            l_len = length;
            if(addr+l_len>m_length) 
                l_len = m_length - addr;
            memcpy(data,&m_datastr[addr],l_len);
        }
    }    
    return l_len;    
}

void fst_controller::Servconf::Saveconf(void)
{
    if ((NULL != m_datastr)&&(NULL != p_paramgroup_))
    {
        std::vector<int> stored_conf;
        stored_conf.resize(m_stored_length/4);
        if(m_startaddr_stored+m_stored_length<=m_length)
        {
            for(int i = 0;i<m_stored_length/4;i++)
            {
            	stored_conf[i] = *(int *)&m_datastr[i*4+m_startaddr_stored];
            }
        }
        p_paramgroup_->setParam("servo.stored_param", stored_conf);
        p_paramgroup_->dumpParamFile(m_filename);
    }
}


void fst_controller::Servconf::DownloadConf(fst_controller::ServoService &serv,unsigned int 
addr,int length)
{
    ServiceRequest client_service_request;
    char *data = new char[length];
    int l_len = Getconf(addr, data,fst_controller::ServoService::SERVO_CONF_SEG);
    if(l_len>0)
    {
        serv.DownloadParam(addr,data,l_len);
    }
    delete [] data;
}


int fst_controller::Servconf::UploadConf(fst_controller::ServoService &serv,int addr,int 
length)
{
    char *data = new char[length];
    ERROR_CODE_TYPE err = serv.UploadParam(addr,data,length);
    if(FST_SUCCESS==err)
    {
       Setconf(addr,data,length);
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

void fst_controller::Servconf::InitDownloadConf(fst_controller::ServoService &serv)
{
    int l_len;
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    for(int addr = m_startaddr_stored;addr<m_startaddr_stored+m_stored_length;)
    {
        l_len = m_startaddr_stored+m_stored_length-addr;
        l_len = (l_len>max_seg_len)?max_seg_len:l_len;

        DownloadConf(serv, addr,l_len);
        addr += l_len;
    }	
}

void fst_controller::Servconf::InitConfFile(fst_controller::ServoService &serv)
{
    int l_len;
    int max_seg_len = fst_controller::ServoService::SERVO_CONF_SEG;
    for(int addr = m_startaddr_stored;addr<m_startaddr_stored+m_stored_length;)
    {
        l_len = m_startaddr_stored+m_stored_length-addr;
        l_len = (l_len>max_seg_len)?max_seg_len:l_len;

        UploadConf(serv, addr,l_len);
        addr += l_len;
    }	
    Saveconf();
}



