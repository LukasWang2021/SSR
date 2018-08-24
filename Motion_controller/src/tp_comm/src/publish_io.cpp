#include "tp_comm.h"

using namespace fst_comm;
using namespace std;


void TpComm::handlePublishElementIo(Comm_Publish& package, int element_index, TpPublishElement& list_element)
{
    if(!encodePublishElement(package.element[element_index].data, MessageType_Int32_fields, list_element.data_ptr))
    {
        FST_ERROR("handlePublishElementIo: failed to encode IO.");
    }
}