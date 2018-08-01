#include "tp_comm.h"
#include "common/common.h"

using namespace std;

// “comm/basetype/int32”
void TpComm::handlePublishElement0x7622aa34(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x7622aa34: failed to encode /comm/basetype/int32");
    }
}

void TpComm::handlePublishElement0x00011423(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00011423: failed to encode ...");
    }
}

void TpComm::handlePublishElement0x00010363(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00010363: failed to encode ...");
    }
}

void TpComm::handlePublishElement0x00001F13(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00001F13: failed to encode ...");
    }
}

void TpComm::handlePublishElement0x0000D175(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0000D175: failed to encode ...");
    }
}

void TpComm::handlePublishElement0x00015453(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_String_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x00015453: failed to encode ...");
    }
}

void TpComm::handlePublishElement0x0000AB25(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x0000AB25: failed to encode ...");
    }
}


