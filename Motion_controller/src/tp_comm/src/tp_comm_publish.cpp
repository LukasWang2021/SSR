#include "tp_comm_component.h"
#include "common/common.h"


using namespace std;


// “comm/basetype/int32”
void TpCommComponent::handlePublishElement0x7622aa34(comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, comm_MessageTypeInt32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElement0x7622aa34: failed to encode /comm/basetype/int32");
    }
}

void TpCommComponent::handlePublishElement0x00011423(comm_Publish& package, int element_index, TpPublishElement& list_element){}
void TpCommComponent::handlePublishElement0x00010363(comm_Publish& package, int element_index, TpPublishElement& list_element){}
void TpCommComponent::handlePublishElement0x00001F13(comm_Publish& package, int element_index, TpPublishElement& list_element){}
void TpCommComponent::handlePublishElement0x0000D175(comm_Publish& package, int element_index, TpPublishElement& list_element){}
void TpCommComponent::handlePublishElement0x00015453(comm_Publish& package, int element_index, TpPublishElement& list_element){}
void TpCommComponent::handlePublishElement0x0000AB25(comm_Publish& package, int element_index, TpPublishElement& list_element){}


