#include "tp_comm.h"

using namespace fst_comm;
using namespace std;


void TpComm::handlePublishElementRegPr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_PrValue_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegPr: failed to encode Pr");
    }
}

void TpComm::handlePublishElementRegSr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_String_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegSr: failed to encode Sr");
    }
}

void TpComm::handlePublishElementRegMr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegMr: failed to encode Mr");
    }
}

void TpComm::handlePublishElementRegHr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_HrValue_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegHr: failed to encode Hr");
    }
}

void TpComm::handlePublishElementRegR(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Double_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegR: failed to encode R");
    }
}
