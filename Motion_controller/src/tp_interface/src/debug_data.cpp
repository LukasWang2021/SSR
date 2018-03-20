#include "debug_data.h"
#include "common.h"



DebugData::DebugData()
{
  //  fifo1_.count = 0;
  //  fifo2_.count = 0;

 //   fifo1_added_ = false;
   // fifo2_added_ = false;
}

DebugData::~DebugData()
{

}
#if 0
void DebugData::fillF1Data(fst_controller::PoseEuler *pose)
{
    std::lock_guard<std::mutex> lock(fifo1_.mtx);
    int count = fifo1_.count++;
    if (count >= MAX_FIFO_LEN)
    {
        FST_INFO("fifo2_ full");
        fifo1_.count = 0;
        return;
    }
     
    memcpy(fifo1_.point[count].data, pose, sizeof(Point));
}

void DebugData::fillF1Data(fst_controller::Joint *jnt)
{
    std::lock_guard<std::mutex> lock(fifo1_.mtx);
    int count = fifo1_.count++;
    if (count >= MAX_FIFO_LEN)
    {
        FST_INFO("fifo1_ full");
        fifo1_.count = 0;
        return;
    }
     
    memcpy(fifo1_.point[count].data, jnt, sizeof(Point)); 
}

void DebugData::fillF2Data(fst_controller::Joint *jnt)
{
    std::lock_guard<std::mutex> lock(fifo2_.mtx);
    int count = fifo2_.count++;
    if (count >= MAX_FIFO_LEN)
    {
        FST_INFO("fifo2_ full");
        fifo2_.count = 0;
        return;
    }
    memcpy(fifo2_.point[count].data, jnt, sizeof(Point));
}

void DebugData::clearFIFO1()
{
    std::lock_guard<std::mutex> lock(fifo1_.mtx);
    fifo1_.count = 0;
}

void DebugData::clearFIFO2()
{
    std::lock_guard<std::mutex> lock(fifo2_.mtx);
    fifo2_.count = 0;
}

void DebugData::addFIFO1()
{
    fifo1_added_ = true;
}
void DebugData::addFIFO2()
{
    fifo2_added_ = true;
}
void DebugData::removeFIFO1()
{
    fifo1_added_ = false;
}
void DebugData::removeFIFO2()
{
    fifo2_added_ = false;
}

bool DebugData::loadF1Data(int id, motion_spec_FIFOData *fifo)
{
    std::lock_guard<std::mutex> lock(fifo1_.mtx);
    if (fifo1_.count <= 0)
        return false;

    fifo->id = id;    
    fifo->data.size = fifo1_.count * sizeof(Point);  
    memcpy(fifo->data.bytes, (char*)fifo1_.point, fifo->data.size);
    fifo1_.count = 0;

    return true;
}

bool DebugData::loadF2Data(int id, motion_spec_FIFOData *fifo)
{
    std::lock_guard<std::mutex> lock(fifo2_.mtx);
    if (fifo2_.count <= 0)
        return false;

    fifo->id = id;    
    fifo->data.size = fifo2_.count * sizeof(Point);  
    memcpy(fifo->data.bytes, (char*)fifo2_.point, fifo->data.size);
    fifo2_.count = 0;

    return true;
}
#endif
