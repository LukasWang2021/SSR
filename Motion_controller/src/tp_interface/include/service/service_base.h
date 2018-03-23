/**
 * @file service_base.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-26
 */

#ifndef SERVICE_BASE_H_
#define SERVICE_BASE_H_

#include <string.h>
#include "common.h"
#include "error_monitor.h"
#include "error_code.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include "service_actions/response_actions.h"
#include "comm_interface/comm_interface.h"

class ServiceBase
{
  public:
    ServiceBase(const char* name)
    {
        U64 result = comm_.createChannel(COMM_REQ, COMM_IPC, name);
        if (result != TPI_SUCCESS)
        {
            FST_INFO("Error: fail to create %s channel.", name);
            rcs::Error::instance()->add(INIT_CONTROLLER_FAILED);
        }
      //  FST_INFO("createChannel:%s", name);
    }
    ~ServiceBase()
    {
    }

    bool sendRequest(ServiceRequest *req)
    {
        if (sendCountOut(req))
        {
            ServiceResponse resp;
            recvTimeout(&resp);
            if (resp.res_id == -1)
                return false;
        }
        return true;
    }

    bool sendCountOut(ServiceRequest *req, int attempts = 100)
    {
        U64 result;
        int count = 0;
        do
        {
            result = comm_.send(req, sizeof(ServiceRequest), COMM_DONTWAIT);
            //FST_INFO("id:%x,buf:%x,%x,%x,%x,%x", req->req_id, *(int*)&req->req_buff[0], *(int*)&req->req_buff[4], *(int*)&req->req_buff[8],*(int*)&req->req_buff[12],*(int*)&req->req_buff[16]);
            ++count;
            if (count > attempts)
            {
                FST_ERROR("send failed heartbeat");
                rcs::Error::instance()->add(WRITE_SERVICE_TIMETOUT);
                return false;
            }
        }while (result != TPI_SUCCESS);
        return true;
    }

    bool recvTimeout(ServiceResponse *resp, int attempts = 100, int interval = 5)
    {
        U64 result;
        int count = 0;
        do
        {
            usleep(5*1000);
            result = comm_.recv(resp, sizeof(ServiceResponse), COMM_DONTWAIT);
           // FST_INFO("++++++recv result:%llx", result);
            ++count;
            if (count > attempts)
            {
                FST_ERROR("service recv timeout:%d", count);
                rcs::Error::instance()->add(READ_SERVICE_TIMEOUT);
                return false;
            }
            //FST_INFO("result:%llx", result);
        }while (result != TPI_SUCCESS);
        return true;
    }

  private:
    fst_comm_interface::CommInterface comm_;
};

#endif
