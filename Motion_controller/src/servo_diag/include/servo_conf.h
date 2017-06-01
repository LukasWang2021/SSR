/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        servo_conf.h
  Author:      Yan He
  Data:        Dec.1  2016
  Modify:      Dec.23 2016
  Description: Servconf--Class declaration.
**********************************************************************/


#ifndef SERVO_CONF_H
#define SERVO_CONF_H

#include <string>
#include <parameter_manager/parameter_manager_param_group.h>
#include <servo_service.h>

namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.
class Servconf {
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    Servconf
    // Summary: The constructor of class
    // In:      filename-> configuration data file name;length-> configuration data length
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    Servconf(const std::string &filename);


    //------------------------------------------------------------
    // Function:    ~Servconf
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~Servconf();


    //------------------------------------------------------------
    // Function:    Setconf
    // Summary: To set the configuration Data.
    // In:      addr-> data start address; data->data to be set in conf; length -> length of data
    // Out:     None
    // Return:  Set length
    //------------------------------------------------------------
    int Setconf(unsigned int addr, const char *data, int length);


    //------------------------------------------------------------
    // Function:    Getconf
    // Summary: To Get the configuration Data.
    // In:      addr-> data start address;  length -> length of data
    // Out:     data->data buf stored the configuration data;
    // Return:  Get length
    //------------------------------------------------------------
    int Getconf(unsigned int addr, char *data, int length);


    //------------------------------------------------------------
    // Function:    Saveconf
    // Summary: To Save configuration Data to file.
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void Saveconf(void);
    
    //------------------------------------------------------------
    // Function:   DownloadConf
    // Summary: To Download configuration Data to Core1.
    // In:     core1 service interface, address ,length
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void DownloadConf(fst_controller::ServoService &serv,unsigned int addr,int length);
    
    
    //------------------------------------------------------------
    // Function:   UploadConf
    // Summary: To upload configuration Data from Core1.
    // In:      core1 service interface, address ,lengt
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    int UploadConf(fst_controller::ServoService &serv,int addr,int length);


    //------------------------------------------------------------
    // Function:   InitDownloadConf
    // Summary:  To Initial Download all configuration Data to Core1.
    // In:      core1 service interface
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void InitDownloadConf(fst_controller::ServoService &serv);    

    //------------------------------------------------------------
    // Function:   InitConfFile
    // Summary:  Initialize conf file, ONLY for initial Conf FILE GENERATION
    // In:      core1 service interface
    // Out:     None
    // Return:  None
    //------------------------------------------------------------

    void InitConfFile(fst_controller::ServoService &serv);

    // -----------------------------private functions---------------------------------------------

  private:

    // -----------------------------member variables---------------------------------------------


    char * m_datastr;                    
    int    m_length;
    int    m_startaddr_stored;
    int    m_stored_length;
    std::string m_filename;
    fst_parameter::ParamGroup * p_paramgroup_;
    const int SERVO_CONF_SEG_SIZE = 512;

};  // class Servconf
}   // namespace fst_controller


#endif  // #ifndef SERVO_CONF_H
