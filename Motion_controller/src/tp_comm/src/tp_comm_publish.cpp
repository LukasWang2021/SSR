#include "tp_comm.h"

using namespace fst_comm;
using namespace std;

// UserOpMode, MessageType_Int32
void TpComm::handlePublishElement0x00015255(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode UserOpMode");
    }
}

//RunningStatus, MessageType_Int32
void TpComm::handlePublishElement0x00001F33(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode RunningStatus");
    }
}

//InterpreterStatus, MessageType_Int32
void TpComm::handlePublishElement0x00003203(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode InterpreterStatus");
    }
}

// RobotStatus, MessageType_Int32
void TpComm::handlePublishElement0x00012943(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode RobotStatus");
    }
}

// CtrlStatus, MessageType_Int32
void TpComm::handlePublishElement0x0000E8E3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode RobotStatus");
    }
}

//ServoStatus, MessageType_Int32
void TpComm::handlePublishElement0x00002053(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode ServoStatus");
    }
}

//SafetyAlarm, MessageType_Int32
void TpComm::handlePublishElement0x0000D0AD(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015255: failed to encode SafetyAlarm");
    }
}

void TpComm::handlePublishElement0x00013643(Comm_Publish& package, int element_index, TpPublishElement& list_element){}
