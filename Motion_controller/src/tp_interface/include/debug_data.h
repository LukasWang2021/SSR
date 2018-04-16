/**
 * @file debug_data.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-10-31
 */


#ifndef DEBUG_DATA_H_
#define DEBUG_DATA_H_

#include <mutex>
#include <atomic>
#include "base.h"
#include "motionSL.pb.h"
#include "fst_datatype.h"

#define MAX_FIFO_LEN    (1024)

typedef struct _Point
{
    double data[6];
}Point;

typedef struct _FIFOData
{
    std::mutex  mtx;
    int         count; 
    Point       point[MAX_FIFO_LEN];
}FIFOData;


class DebugData
{
  public:
    DebugData();
    ~DebugData();

    void fillF1Data(fst_controller::PoseEuler *pose);
    void fillF1Data(fst_controller::Joint *jnt);
    void fillF2Data(fst_controller::Joint *jnt);
    void clearFIFO1();
    void clearFIFO2();

    void addFIFO1();
    void addFIFO2();
    void removeFIFO1();
    void removeFIFO2();

  //  bool loadF1Data(int id, motion_spec_FIFOData *fifo);
   // bool loadF2Data(int id, motion_spec_FIFOData *fifo);


  private:
   // FIFOData fifo1_;
   // FIFOData fifo2_;

    std::atomic_bool fifo1_added_;
    std::atomic_bool fifo2_added_;
};

#endif //#ifndef DEBUG_DATA_H_
