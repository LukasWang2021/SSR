/**
 * @file service_parameter.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-12-27
 */


#ifndef SERVICE_PARAMETER_H_
#define SERVICE_PARAMETER_H_
#include "service_base.h"
#include <vector>

using std::vector;

class ServiceParam: public ServiceBase
{
  public:
    ServiceParam():ServiceBase("servo_param")
    {
    }
    ~ServiceParam()
    {
    }

    static ServiceParam* instance();

    bool setConfigData(int id, const vector<double> &data);
    bool getConfigData(int id, vector<double> &data);

  //private:
};


#endif  //#ifndef SERVICE_PARAMETER_H_
