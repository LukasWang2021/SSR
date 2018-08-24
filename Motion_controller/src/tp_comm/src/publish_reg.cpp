#include "tp_comm.h"

using namespace fst_comm;
using namespace std;


void TpComm::handlePublishElementRegPr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_PrRegData_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegPr: failed to encode Pr");
    }
}

void TpComm::handlePublishElementRegSr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_SrRegData_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegSr: failed to encode Sr");
    }
}

void TpComm::handlePublishElementRegMr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_MrRegData_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegMr: failed to encode Mr");
    }
}

void TpComm::handlePublishElementRegHr(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_HrRegData_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegHr: failed to encode Hr");
    }
}

void TpComm::handlePublishElementRegR(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_RRegData_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementRegR: failed to encode R");
    }
}
