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
        FST_ERROR("handlePublishElement0x00001F33: failed to encode RunningStatus");
    }
}

//InterpreterStatus, MessageType_Int32
void TpComm::handlePublishElement0x00003203(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00003203: failed to encode InterpreterStatus");
    }
}

// RobotStatus, MessageType_Int32
void TpComm::handlePublishElement0x00012943(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00012943: failed to encode RobotStatus");
    }
}

// CtrlStatus, MessageType_Int32
void TpComm::handlePublishElement0x0000E8E3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0000E8E3: failed to encode CtrlStatus");
    }
}

//ServoStatus, MessageType_Int32
void TpComm::handlePublishElement0x00002053(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00002053: failed to encode ServoStatus");
    }
}

//SafetyAlarm, MessageType_Int32
void TpComm::handlePublishElement0x0000D0AD(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0000D0AD: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/axis_group/feedback/joints"
void TpComm::handlePublishElement0x000161F3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_DoubleList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x000161F3: failed to encode motion_control axis_group feedback joints");
    }
}

//"/publish/motion_control/axis_group/feedback/tcp_world_cartesian"
void TpComm::handlePublishElement0x00009D8E(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_DoubleList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegR: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/axis_group/feedback/tcp_base_cartesian"
void TpComm::handlePublishElement0x00002D5E(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_DoubleList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00002D5E: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/axis_group/feedback/tcp_current_cartesian"
void TpComm::handlePublishElement0x0000352E(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_DoubleList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0000352E: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/axis_group/current_coordinate"
void TpComm::handlePublishElement0x00012C55(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32List_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00012C55: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/axis_group/current_tool"
void TpComm::handlePublishElement0x00004BEC(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32List_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00004BEC: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/global_vel_ratio",	0x00012A4F,	"MessageType_Double",
void TpComm::handlePublishElement0x00012A4F(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Double_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00012A4F: failed to encode SafetyAlarm");
    }
}

//"/publish/motion_control/global_acc_ratio",	0x0001517F,	"MessageType_Double",
void TpComm::handlePublishElement0x0001517F(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Double_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0001517F: failed to encode SafetyAlarm");
    }
}

//"/publish/interpreter/program_status"
void TpComm::handlePublishElement0x00001AF3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_String_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00001AF3: failed to encode SafetyAlarm");
    }
}

//"/publish/interpreter/tp_program_status"
void TpComm::handlePublishElement0x000042B3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_StringList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x000042B3: failed to encode SafetyAlarm");
    }
}

//"/publish/controller/safety_board_status"
void TpComm::handlePublishElement0x000123C3(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Uint32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x000123C3: failed to encode");
    }
}

//"/publish/controller/io_board_status"
void TpComm::handlePublishElement0x00006D93(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_IoBoardStatusList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00006D93: failed to encode");
    }
}

void TpComm::handlePublishElement0x00011843(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_ModbusClientCtrlStatusList_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00006D93: failed to encode");
    }
}
