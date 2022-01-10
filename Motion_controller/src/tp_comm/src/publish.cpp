#include "tp_comm.h"

using namespace user_space;
using namespace std;
using namespace log_space;

/********publish/axes_feedback, MessageType_AxisFeedbackList(count=14)**********/   
void TpComm::handlePublishElement0x0001715B(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{ 
    if(!encodePublishElement(package.element[element_index].data, MessageType_AxisFeedbackList_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x0001715B: failed to encode");
    }
}

/********publish/servo1000/servos_feedback, MessageType_ServoFeedbackList(count=14)**********/
void TpComm::handlePublishElement0x0001128B(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Servo1000ServoFeedbackList_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x0000128B: failed to encode");
    }
}

/********publish/servo1000/cpu_feedback, MessageType_Uint32List(count=15)**********/ 
void TpComm::handlePublishElement0x00012FFB(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Uint32List_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x00012FCB: failed to encode");
    }
}

/********publish/io1000/io_feedback, MessageType_Uint32List(count=4)**********/	
void TpComm::handlePublishElement0x00013C8B(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Uint32List_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x00013C8B: failed to encode");
    }
}

/********publish/ioAnalog/io_analog_feedback, MessageType_Uint32List(count=12)**********/
void TpComm::handlePublishElement0x00007C5B(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Uint32List_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x00007C5B: failed to encode");
    }
}

/********publish/iosafety/safety_feedback, MessageType_Uint32List(count=2)**********/	
void TpComm::handlePublishElement0x0001472B(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Uint32List_fields, list_element.data_ptr))
    {
        LogProducer::error("publish", "handlePublishElement0x0001472B: failed to encode");
    }
}