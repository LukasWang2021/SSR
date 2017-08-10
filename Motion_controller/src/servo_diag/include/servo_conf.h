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
    // Function:    setConf
    // Summary: To set the configuration Data.
    // In:      addr-> data start address; data->data to be set in conf; length -> length of data
    // Out:     None
    // Return:  Set length
    //------------------------------------------------------------
    int setConf(unsigned int addr, const char *data, int length);


    //------------------------------------------------------------
    // Function:    getConf
    // Summary: To Get the configuration Data.
    // In:      addr-> data start address;  length -> length of data
    // Out:     data->data buf stored the configuration data;
    // Return:  Get length
    //------------------------------------------------------------
    int getConf(unsigned int addr, char *data, int length);


    //------------------------------------------------------------
    // Function:    saveConf
    // Summary: To Save configuration Data to file.
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void saveConf(void);
    
    //------------------------------------------------------------
    // Function:   downloadConf
    // Summary: To Download configuration Data to Core1.
    // In:     core1 service interface, address ,length
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void downloadConf(fst_controller::ServoService &serv,unsigned int addr,int length);
    
    
    //------------------------------------------------------------
    // Function:   uploadConf
    // Summary: To upload configuration Data from Core1.
    // In:      core1 service interface, address ,lengt
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    int uploadConf(fst_controller::ServoService &serv,int addr,int length);


    //------------------------------------------------------------
    // Function:   initDownloadConf
    // Summary:  To Initial Download all configuration Data to Core1.
    // In:      core1 service interface
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void initDownloadConf(fst_controller::ServoService &serv);    

    //------------------------------------------------------------
    // Function:   initConfFile
    // Summary:  Initialize conf file, ONLY for initial Conf FILE GENERATION
    // In:      core1 service interface
    // Out:     None
    // Return:  None
    //------------------------------------------------------------

    void initConfFile(fst_controller::ServoService &serv);

    // -----------------------------private functions---------------------------------------------

  private:

    // -----------------------------member variables---------------------------------------------


    char * datastr_;                    
    int    paramlength_;
    int    startaddr_stored_;
    int    stored_length_;
    std::string filename_;
    fst_parameter::ParamGroup * p_paramgroup_;
    const int SERVO_CONF_SEG_SIZE = 512;

};  // class Servconf
}   // namespace fst_controller


#endif  // #ifndef SERVO_CONF_H
