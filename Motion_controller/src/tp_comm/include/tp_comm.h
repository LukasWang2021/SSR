#ifndef TP_COMM_H
#define TP_COMM_H

#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <sys/time.h>
#include <nanomsg/nn.h>

#include "protoc.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "thread_help.h"
#include "common_log.h"
#include "local_ip.h"

#include "tp_comm_manager_param.h"

using namespace std;

namespace fst_comm
{
typedef struct
{
    unsigned int hash;
    void* data_ptr;
}TpPublishElement;

typedef struct
{
    unsigned int hash;
    std::vector<TpPublishElement> element_list_;
    Comm_Publish package;
    int interval_min;   // ms
    int interval_max;   // ms
    bool is_element_changed;
    struct timeval last_publish_time;
}TpPublish;

typedef struct
{
    unsigned int hash;
    void* request_data_ptr;
    void* response_data_ptr;
}TpRequestResponse;

class TpComm
{
public:
    TpComm();
    ~TpComm();

    unsigned long long int init();
    unsigned long long int open();
    void close();

    void tpCommThreadFunc();
    bool isRunning();

    bool popTaskFromRequestList(TpRequestResponse* task);
    int32_t getResponseSucceed(void* response_data_ptr);
    void pushTaskToResponseList(TpRequestResponse& package);
    TpPublish generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max);
    void addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr);

    bool isTopicExisted(unsigned int topic_hash);
    void pushTaskToPublishList(TpPublish& package);
    std::vector<TpPublishElement> eraseTaskFromPublishList(unsigned int &topic_hash);
    void lockPublishMutex();
    void unlockPublishMutex();

    bool isRegTopicExisted(unsigned int topic_hash);
    void pushTaskToRegPublishList(TpPublish& package);
    std::vector<TpPublishElement> eraseTaskFromRegPublishList(unsigned int &topic_hash);
    void lockRegPublishMutex();
    void unlockRegPublishMutex();

    bool isIoTopicExisted(unsigned int topic_hash);
    void pushTaskToIoPublishList(TpPublish& package);
    std::vector<TpPublishElement> eraseTaskFromIoPublishList(unsigned int &topic_hash);
    void lockIoPublishMutex();
    void unlockIoPublishMutex();

    // component parameters
    int log_level_;
private:
    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef void (TpComm::*HandleRequestFuncPtr)(int);
    typedef void (TpComm::*HandleResponseFuncPtr)(std::vector<TpRequestResponse>::iterator&, int&);
    typedef void (TpComm::*HandlePublishElementFuncPtr)(Comm_Publish&, int, TpPublishElement&);

    void initRpcTable();
    void initRpcQuickSearchTable();
    void initPublishElementTable();
    void initPublishElementQuickSearchTable();
    bool initComponentParams();
    HandleRequestFuncPtr getRequestHandlerByHash(unsigned int hash);
    HandleResponseFuncPtr getResponseHandlerByHash(unsigned int hash);
    HandlePublishElementFuncPtr getPublishElementHandlerByHash(unsigned int hash);
    Comm_Authority getPublishElementAuthorityByHash(unsigned int hash);
    Comm_Authority getRpcTableElementAuthorityByHash(unsigned int hash);

    void handleRequest();
    void pushTaskToRequestList(unsigned int hash, void* request_data_ptr, void* response_data_ptr);
    void handleResponseList();

    void handlePublishList();
    void handleRegPublishList();
    void handleIoPublishList();

    long long computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val);
    long long computeTimeForTp(struct timeval& current_time_val);
    bool checkPublishCondition(long long time_elapsed, bool is_element_changed, int interval_min, int interval_max);

    bool decodeRequestPackage(const pb_field_t fields[], void* request_data_ptr, int recv_bytes);
    bool checkAuthority(Comm_Authority request_authority, Comm_Authority controller_authority);
    void initResponsePackage(void* request_data_ptr, void* response_data_ptr, int package_left);
    void initCommFailedResponsePackage(void* request_data_ptr, void* response_data_ptr);
    void initDecodeFailedResponsePackage(void* request_data_ptr, void* response_data_ptr);
    void handleRequestPackage(unsigned int hash, void* request_data_ptr, void* response_data_ptr, int recv_bytes, 
                                    const pb_field_t fields[], int package_left);
    ResponseMessageType_Uint64_PublishTable getResponseSucceedPublishTable();

    bool encodeResponsePackage(unsigned int hash, const pb_field_t fields[], void* response_data_ptr, int& send_buffer_size);
    bool encodePublishElement(Comm_PublishElement_data_t& element, const pb_field_t fields[], void* element_data_ptr);
    bool encodePublishPackage(Comm_Publish& package, unsigned int hash, struct timeval& current_time_val, int& send_buffer_size);

    void handlePublishElementRegPr(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    void handlePublishElementRegSr(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    void handlePublishElementRegMr(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    void handlePublishElementRegHr(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    void handlePublishElementRegR(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    void handlePublishElementIo(Comm_Publish& package, int element_index, TpPublishElement& list_element);

    void recordLog(ErrorCode log_code, ErrorCode error_code, std::string rpc_path);
    void handleRequestNonexistentHash(int hash, int recv_bytes);
    void handleResponseNonexistentHash(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********GetUserOpMode, RequestMessageType_Void**********/          
    void handleRequest0x00000C05(int recv_bytes);
    /********GetRunningStatus, RequestMessageType_Void**********/       
    void handleRequest0x00000AB3(int recv_bytes);
    /********GetInterpreterStatus, RequestMessageType_Void**********/   
    void handleRequest0x00016483(int recv_bytes);
    /********GetRobotStatus, RequestMessageType_Void**********/         
    void handleRequest0x00006F83(int recv_bytes);
    /********GetCtrlStatus, RequestMessageType_Void**********/          
    void handleRequest0x0000E9D3(int recv_bytes);
    /********GetServoStatus, RequestMessageType_Void**********/         
    void handleRequest0x0000D113(int recv_bytes);
    /********GetSafetyAlarm, RequestMessageType_Void**********/         
    void handleRequest0x0000C00D(int recv_bytes);
    /********CallEstop, RequestMessageType_Void**********/              
    void handleRequest0x00013940(int recv_bytes);
    /********CallReset, RequestMessageType_Void**********/              
    void handleRequest0x000161E4(int recv_bytes);
    /********SetUserOpMode, RequestMessageType_Int32**********/         
    void handleRequest0x00002ED5(int recv_bytes);
    /********rpc/controller/shutdown, RequestMessageType_Void**********/
    void handleRequest0x0000899E(int recv_bytes);
    /********rpc/controller/setSystemTime, RequestMessageType_Uint64**********/
    void handleRequest0x000167C5(int recv_bytes);
    /********rpc/controller/getSystemTime, RequestMessageType_Void**********/
    void handleRequest0x000003F5(int recv_bytes);
    /********rpc/controller/getVersion, RequestMessageType_Void**********/
    void handleRequest0x000093EE(int recv_bytes);
    /********rpc/interpreter/switchStep, RequestMessageType_Int32**********/
    void handleRequest0x000140F0(int recv_bytes);

    /********tool_manager/addTool, RequestMessageType_ToolInfo**********/
    void handleRequest0x0000A22C(int recv_bytes);
    /********tool_manager/deleteTool, RequestMessageType_Int32**********/
    void handleRequest0x00010E4C(int recv_bytes);
    /********tool_manager/updateTool, RequestMessageType_ToolInfo**********/   
    void handleRequest0x0000C78C(int recv_bytes);
    /********tool_manager/moveTool, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x000085FC(int recv_bytes);
    /********tool_manager/getToolInfoById, RequestMessageType_Int32**********/
    void handleRequest0x00009E34(int recv_bytes);
    /********tool_manager/getAllValidToolSummaryInfo, RequestMessageType_Void**********/
    void handleRequest0x0001104F(int recv_bytes);

    /********coordinate_manager/addUserCoord, RequestMessageType_UserInfo**********/
    void handleRequest0x00016764(int recv_bytes);
    /********coordinate_manager/deleteUserCoord, RequestMessageType_Int32**********/
    void handleRequest0x0000BAF4(int recv_bytes);
    /********coordinate_manager/updateUserCoord, RequestMessageType_UserInfo**********/
    void handleRequest0x0000EC14(int recv_bytes);
    /********coordinate_manager/moveUserCoord, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000E104(int recv_bytes);
    /********coordinate_manager/getUserCoordInfoById, RequestMessageType_Int32**********/
    void handleRequest0x00004324(int recv_bytes);
    /********coordinate_manager/getAllValidUserCoordSummaryInfo, RequestMessageType_Void**********/
    void handleRequest0x0001838F(int recv_bytes);
    /********getRpcTable, RequestMessageType_Void**********/
    void handleRequest0x00004FA5(int recv_bytes);
    /********getPublishTable, RequestMessageType_Void**********/
    void handleRequest0x000147A5(int recv_bytes);

    /********rpc/reg_manager/r/addReg, RequestMessageType_RRegData**********/
    void handleRequest0x00004FF7(int recv_bytes);
    /********rpc/reg_manager/r/deleteReg, RequestMessageType_Int32**********/
    void handleRequest0x000012F7(int recv_bytes);
    /********rpc/reg_manager/r/updateReg, RequestMessageType_RRegData**********/
    void handleRequest0x00005757(int recv_bytes);
    /********rpc/reg_manager/r/getReg, RequestMessageType_Int32**********/
    void handleRequest0x0000EAB7(int recv_bytes);
    /********rpc/reg_manager/r/moveReg, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000C877(int recv_bytes);
    /********rpc/reg_manager/r/getChangedList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000A904(int recv_bytes);
    /********rpc/reg_manager/r/getValidList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00008CE4(int recv_bytes);
    /********rpc/reg_manager/mr/addReg, RequestMessageType_MrRegData**********/
    void handleRequest0x000097E7(int recv_bytes);
    /********rpc/reg_manager/mr/deleteReg, RequestMessageType_Int32**********/
    void handleRequest0x0000E5D7(int recv_bytes);
    /********rpc/reg_manager/mr/updateReg, RequestMessageType_MrRegData**********/
    void handleRequest0x0000E9B7(int recv_bytes);
    /********rpc/reg_manager/mr/getReg, RequestMessageType_Int32**********/
    void handleRequest0x0000B507(int recv_bytes);
    /********rpc/reg_manager/mr/moveReg, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00015BA7(int recv_bytes);
    /********rpc/reg_manager/mr/getChangedList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00001774(int recv_bytes);
    /********rpc/reg_manager/mr/getValidList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00015CF4(int recv_bytes);
    /********rpc/reg_manager/sr/addReg, RequestMessageType_SrRegData**********/
    void handleRequest0x000161E7(int recv_bytes);
    /********rpc/reg_manager/sr/deleteReg, RequestMessageType_Int32**********/
    void handleRequest0x0000B817(int recv_bytes);
    /********rpc/reg_manager/sr/updateReg, RequestMessageType_SrRegData**********/
    void handleRequest0x000119F7(int recv_bytes);
    /********rpc/reg_manager/sr/getReg, RequestMessageType_Int32**********/
    void handleRequest0x00017F07(int recv_bytes);
    /********rpc/reg_manager/sr/moveReg, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00002127(int recv_bytes);
    /********rpc/reg_manager/sr/getChangedList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00004834(int recv_bytes);
    /********rpc/reg_manager/sr/getValidList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00009854(int recv_bytes);
    /********rpc/reg_manager/pr/addReg, RequestMessageType_PrRegData**********/
    void handleRequest0x000154E7(int recv_bytes);
    /********rpc/reg_manager/pr/deleteReg, RequestMessageType_Int32**********/
    void handleRequest0x00001097(int recv_bytes);
    /********rpc/reg_manager/pr/updateReg, RequestMessageType_PrRegData**********/
    void handleRequest0x00009EF7(int recv_bytes);
    /********rpc/reg_manager/pr/getReg, RequestMessageType_Int32**********/
    void handleRequest0x00017207(int recv_bytes);
    /********rpc/reg_manager/pr/moveReg, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000D7C7(int recv_bytes);
    /********rpc/reg_manager/pr/getChangedList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000B454(int recv_bytes);
    /********rpc/reg_manager/pr/getValidList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00009354(int recv_bytes);
    /********rpc/reg_manager/hr/addReg, RequestMessageType_HrRegData**********/
    void handleRequest0x00016CE7(int recv_bytes);
    /********rpc/reg_manager/hr/deleteReg, RequestMessageType_Int32**********/
    void handleRequest0x00003D17(int recv_bytes);
    /********rpc/reg_manager/hr/updateReg, RequestMessageType_HrRegData**********/
    void handleRequest0x0000CB77(int recv_bytes);
    /********rpc/reg_manager/hr/getReg, RequestMessageType_Int32**********/
    void handleRequest0x00000367(int recv_bytes);
    /********rpc/reg_manager/hr/moveReg, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00014A87(int recv_bytes);
    /********rpc/reg_manager/hr/getChangedList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00012974(int recv_bytes);
    /********rpc/reg_manager/hr/getValidList, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00006B54(int recv_bytes);

    /********rpc/motion_control/stop, RequestMessageType_Void**********/
    void handleRequest0x00001E70(int recv_bytes);
    /********rpc/motion_control/reset, RequestMessageType_Void**********/
    void handleRequest0x00001D14(int recv_bytes);
    /********rpc/motion_control/axis_group/doStepManualMove, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x000085D5(int recv_bytes);
    /********rpc/motion_control/axis_group/doContinuousManualMove, RequestMessageType_Int32_Int32List(count = 9)**********/
    void handleRequest0x0000D3F5(int recv_bytes);
    /********rpc/motion_control/axis_group/getJointsFeedBack, RequestMessageType_Int32**********/
    void handleRequest0x0000DFBB(int recv_bytes);

    /********rpc/motion_control/axis_group/setUserSoftLimit, RequestMessageType_Int32_JointLimit**********/
    void handleRequest0x000114A4(int recv_bytes);
    /********rpc/motion_control/axis_group/getUserSoftLimit, RequestMessageType_Int32**********/
    void handleRequest0x0000C764(int recv_bytes);
    /********rpc/motion_control/axis_group/setManuSoftLimit, RequestMessageType_Int32_JointLimit**********/
    void handleRequest0x000108E4(int recv_bytes);
    /********rpc/motion_control/axis_group/getManuSoftLimit, RequestMessageType_Int32**********/
    void handleRequest0x0000C244(int recv_bytes);
    /********rpc/motion_control/axis_group/doGotoCartesianPointManualMove, RequestMessageType_Int32_DoubleList(count = 6) **********/
    void handleRequest0x00010C05(int recv_bytes);
    /********rpc/motion_control/axis_group/doGotoJointPointManualMove, RequestMessageType_Int32_DoubleList(count = 9) **********/
    void handleRequest0x00008075(int recv_bytes);
    /********rpc/motion_control/axis_group/doManualStop, RequestMessageType_Int32**********/
    void handleRequest0x0000A9A0(int recv_bytes);
    /********rpc/motion_control/axis_group/setOrientationManualStep, RequestMessageType_Int32_Double**********/
    void handleRequest0x00002940(int recv_bytes);
    /********rpc/motion_control/axis_group/getOrientationManualStep, RequestMessageType_Int32**********/
    void handleRequest0x00016D20(int recv_bytes);
    /********rpc/motion_control/axis_group/setSingleZeroPointOffset, RequestMessageType_Int32List_Double(count=2)**********/
    void handleRequest0x00012404(int recv_bytes);

    /********rpc/interpreter/start, RequestMessageType_String**********/
    void handleRequest0x00006154(int recv_bytes);
    /********rpc/interpreter/debug, RequestMessageType_String**********/
    void handleRequest0x000102D7(int recv_bytes);
    /********rpc/interpreter/forward, RequestMessageType_Void**********/
    void handleRequest0x0000D974(int recv_bytes);
    /********rpc/interpreter/backward, RequestMessageType_Void**********/
    void handleRequest0x00008E74(int recv_bytes);
    /********rpc/interpreter/jump, RequestMessageType_Int32**********/
    void handleRequest0x00015930(int recv_bytes);
    /********rpc/interpreter/pause, RequestMessageType_Void**********/
    void handleRequest0x0000BA55(int recv_bytes);
    /********rpc/interpreter/resume, RequestMessageType_Void**********/
    void handleRequest0x0000CF55(int recv_bytes);
    /********rpc/interpreter/abort, RequestMessageType_Void**********/
    void handleRequest0x000086F4(int recv_bytes);
    /********rpc/controller/setStartMode, RequestMessageType_Int32**********/
    void handleRequest0x00010225(int recv_bytes);
    /********rpc/controller/getStartMode, RequestMessageType_Void**********/
    void handleRequest0x000092E5(int recv_bytes);

    /********rpc/publish/addTopic, RequestMessageType_Topic**********/
    void handleRequest0x000050E3(int recv_bytes);
    /********rpc/publish/deleteTopic, RequestMessageType_UnsignedInt32**********/
    void handleRequest0x00004403(int recv_bytes);
    /********rpc/publish/addRegTopic, RequestMessageType_Topic**********/
    void handleRequest0x000163A3(int recv_bytes);
    /********rpc/publish/addIoTopic, RequestMessageType_Topic**********/
    void handleRequest0x000058F3(int recv_bytes);
    /********rpc/publish/deleteRegTopic, RequestMessageType_Uint32**********/
    void handleRequest0x00010353(int recv_bytes);
    /********rpc/publish/deleteIoTopic, RequestMessageType_Uint32**********/
    void handleRequest0x0000DD03(int recv_bytes);


    /********rpc/motion_control/setGlobalVelRatio, RequestMessageType_Double**********/
    void handleRequest0x000005EF(int recv_bytes);
    /********rpc/motion_control/getGlobalVelRatio, RequestMessageType_Void**********/
    void handleRequest0x0001578F(int recv_bytes);
    /********rpc/motion_control/setGlobalAccRatio, RequestMessageType_Double**********/
    void handleRequest0x0000271F(int recv_bytes);
    /********rpc/motion_control/getGlobalAccRatio, RequestMessageType_Void**********/
    void handleRequest0x00016D9F(int recv_bytes);
    /********rpc/motion_control/axis_group/setHardLimit, RequestMessageType_Int32_JointLimit **********/
    void handleRequest0x0000C454(int recv_bytes);
    /********rpc/motion_control/axis_group/getHardLimit, RequestMessageType_Int32**********/
    void handleRequest0x00013394(int recv_bytes);
    /********rpc/motion_control/axis_group/getCoordinate, RequestMessageType_Int32**********/
    void handleRequest0x00008595(int recv_bytes);
    /********rpc/motion_control/axis_group/setCoordinate, RequestMessageType_Int32List(count = 3) **********/
    void handleRequest0x0000A845(int recv_bytes);
    //"/rpc/motion_control/axis_group/getUserCoordId"
    void handleRequest0x00005BB4(int recv_bytes);
    //"/rpc/motion_control/axis_group/setUserCoordId"
    void handleRequest0x00005CF4(int recv_bytes);
    /********rpc/motion_control/axis_group/getTool, RequestMessageType_Int32**********/
    void handleRequest0x0001354C(int recv_bytes);
    /********rpc/motion_control/axis_group/setTool, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0001581C(int recv_bytes);
    /********rpc/motion_control/axis_group/convertCartToJoint, RequestMessageType_Int32List_DoubleList(Int32List_count=4,DoubleList_count=6)**********/
    void handleRequest0x00010FD4(int recv_bytes);
    /********rpc/motion_control/axis_group/convertJointToCart, RequestMessageType_Int32List_DoubleList(Int32List_count=4,DoubleList_count = 9)**********/
    void handleRequest0x0000B6D4(int recv_bytes);

    /********rpc/motion_control/axis_group/ignoreLostZeroError, RequestMessageType_Int32**********/
    void handleRequest0x00014952(int recv_bytes);
    /********rpc/motion_control/axis_group/setAllZeroPointOffsets, RequestMessageType_Int32_DoubleList(count=9)**********/
    void handleRequest0x00008AB4(int recv_bytes);
    /********rpc/motion_control/axis_group/getAllZeroPointOffsets, RequestMessageType_Int32**********/
    void handleRequest0x00012353(int recv_bytes);
    /********rpc/motion_control/axis_group/getAllZeroErrorMaskStatus, RequestMessageType_Int32**********/
    void handleRequest0x0000C183(int recv_bytes);
    /********rpc/motion_control/axis_group/saveAllZeroPointOffsets, RequestMessageType_Int32**********/
    void handleRequest0x000171D3(int recv_bytes);
    /********rpc/motion_control/axis_group/setSingleZeroPointStatus, RequestMessageType_Int32List(count=3)**********/
    void handleRequest0x00010E43(int recv_bytes);
    //"/rpc/motion_control/axis_group/getAllZeroPointStatus"
    void handleRequest0x000102F3(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateAllZeroPointOffsets, RequestMessageType_Int32**********/
    void handleRequest0x00011B03(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateSingleZeroPointOffset, RequestMessageType_Int32List(count=2)**********/
    void handleRequest0x000131D4(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateZeroPointOffsets, RequestMessageType_Int32_Int32List(count=9)**********/
    void handleRequest0x00005AE3(int recv_bytes);
    /********rpc/motion_control/axis_group/isReferencePointExist, RequestMessageType_Int32**********/
    void handleRequest0x0000D344(int recv_bytes);
    /********rpc/motion_control/axis_group/deleteReferencePoint, RequestMessageType_Int32**********/
    void handleRequest0x00008744(int recv_bytes);
    /********rpc/motion_control/axis_group/saveReferencePoint, RequestMessageType_Int32**********/
    void handleRequest0x00006744(int recv_bytes);
    /********rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets, RequestMessageType_Int32**********/
    void handleRequest0x0000E913(int recv_bytes);
    /********rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset, RequestMessageType_Int32List(count=2)**********/
    void handleRequest0x00004754(int recv_bytes);
    /********rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets, RequestMessageType_Int32_Int32List(count=9)**********/
    void handleRequest0x00007EC3(int recv_bytes);
    /********rpc/motion_control/getAxisGroupInfoList, RequestMessageType_Void**********/
    void handleRequest0x00010F54(int recv_bytes);
    /********rpc/motion_control/axis_group/getUserSoftLimitWithUnit, RequestMessageType_Int32**********/
    void handleRequest0x00008ED4(int recv_bytes);
    /********rpc/motion_control/axis_group/getManuSoftLimitWithUnit, RequestMessageType_Int32**********/
    void handleRequest0x000124E4(int recv_bytes);
    /********rpc/motion_control/axis_group/getHardLimitWithUnit, RequestMessageType_Int32**********/
    void handleRequest0x000092B4(int recv_bytes);
    /********rpc/motion_control/axis_group/setRotateManualStep, RequestMessageType_Int32_Double**********/
    void handleRequest0x00005290(int recv_bytes);
    /********rpc/motion_control/axis_group/getRotateManualStep, RequestMessageType_Int32**********/
    void handleRequest0x00003000(int recv_bytes);
    /********rpc/motion_control/axis_group/setPrismaticManualStep, RequestMessageType_Int32_Double**********/
    void handleRequest0x0000B640(int recv_bytes);
    /********rpc/motion_control/axis_group/getPrismaticManualStep, RequestMessageType_Int32**********/
    void handleRequest0x0000FCE0(int recv_bytes);
    /********rpc/motion_control/axis_group/setCartesianManualStep, RequestMessageType_Int32_Double**********/
    void handleRequest0x0000A420(int recv_bytes);
    /********rpc/motion_control/axis_group/getCartesianManualStep, RequestMessageType_Int32**********/
    void handleRequest0x0000EAC0(int recv_bytes);

    /********rpc/io_mapping/getDIByBit, RequestMessageType_Int32**********/
    void handleRequest0x000050B4(int recv_bytes);
    /********rpc/io_mapping/setDIByBit, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00011754(int recv_bytes);
    /********rpc/io_mapping/getDOByBit, RequestMessageType_Int32**********/
    void handleRequest0x00013074(int recv_bytes);
    /********rpc/io_mapping/setDOByBit, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00007074(int recv_bytes);

    /********rpc/io_mapping/getRIByBit, RequestMessageType_Int32**********/
    void handleRequest0x00000684(int recv_bytes);
    /********rpc/io_mapping/setRIByBit, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x0000CD24(int recv_bytes);
    /********rpc/io_mapping/getROByBit, RequestMessageType_Int32**********/
    void handleRequest0x00005BD4(int recv_bytes);
    /********rpc/io_mapping/setROByBit, RequestMessageType_Int32List(count = 2) **********/
    void handleRequest0x00012274(int recv_bytes);
    /********rpc/io_mapping/syncFileIoStatus, RequestMessageType_String**********/
    void handleRequest0x0000BA73(int recv_bytes);
    /********rpc/io_mapping/syncFileIoMapping, RequestMessageType_String**********/
    void handleRequest0x0000C2A7(int recv_bytes);

    /********rpc/device_manager/getDeviceList , RequestMessageType_Void**********/
    void handleRequest0x0000C1E0(int recv_bytes);
    /********rpc/device_manager/get_FRP8A_IoDeviceInfo, RequestMessageType_Int32**********/
    void handleRequest0x00006BAF(int recv_bytes);
    /********rpc/device_manager/getModbusIoDeviceInfo, RequestMessageType_Void**********/
    void handleRequest0x0001421F(int recv_bytes);
    /********rpc/device_manager/getIoDeviceInfoList, RequestMessageType_Void**********/
    void handleRequest0x000024A4(int recv_bytes);

    /********rpc/program_launching/setMethod, RequestMessageType_Int32**********/
    void handleRequest0x00011544(int recv_bytes);
    /********rpc/program_launching/getMethod, RequestMessageType_Void**********/
    void handleRequest0x00010944(int recv_bytes);
    /********rpc/program_launching/syncFileMacroConfig, RequestMessageType_String**********/
    void handleRequest0x00016B27(int recv_bytes);

    /********rpc/modbus/newServer, RequestMessageType_ModbusServerConfig**********/
    void handleRequest0x0000D1B2(int recv_bytes);
    /********rpc/modbus/deleteServer, RequestMessageType_Void**********/
    void handleRequest0x00006C22(int recv_bytes);
    /********rpc/modbus/saveServerConfig, RequestMessageType_ModbusServerConfig**********/
    void handleRequest0x000050E7(int recv_bytes);
    /********rpc/modbus/newClient, RequestMessageType_ModbusClientConfig**********/
    void handleRequest0x00009F84(int recv_bytes);
    /********rpc/modbus/deleteClient, RequestMessageType_Int32**********/
    void handleRequest0x00014CF4(int recv_bytes);
    /********rpc/modbus/saveClientConfig, RequestMessageType_ModbusClientConfig**********/
    void handleRequest0x00002B57(int recv_bytes);
    /********rpc/modbus/getServerConfig, RequestMessageType_Void**********/
    void handleRequest0x00016947(int recv_bytes);
    /********rpc/modbus/getClientConfig, RequestMessageType_Int32**********/
    void handleRequest0x0000FC17(int recv_bytes);
    /********rpc/modbus/getConnectionStatus, RequestMessageType_Void**********/
    void handleRequest0x0000E973(int recv_bytes);
    /********rpc/modbus/setConnectionStatus, RequestMessageType_Bool**********/
    void handleRequest0x00010C63(int recv_bytes);

    /********GetUserOpMode, ResponseMessageType_Int32**********/	
    void handleResponse0x00000C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetRunningStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetInterpreterStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00016483(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetRobotStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x00006F83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetCtrlStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetServoStatus, ResponseMessageType_Int32**********/	
    void handleResponse0x0000D113(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********GetSafetyAlarm, ResponseMessageType_Int32**********/	
    void handleResponse0x0000C00D(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********CallEstop, ResponseMessageType_Bool**********/	        
    void handleResponse0x00013940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********CallReset, ResponseMessageType_Bool**********/	        
    void handleResponse0x000161E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********SetUserOpMode, ResponseMessageType_Bool**********/	    
    void handleResponse0x00002ED5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********addTopic, ResponseMessageType_Bool**********/
    void handleResponse0x00000773(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********deleteTopicResponseMessageType_Bool**********/
    void handleResponse0x0000BB93(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/shutdown, ResponseMessageType_Uint64**********/
    void handleResponse0x0000899E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/setSystemTime, ResponseMessageType_Uint64**********/
    void handleResponse0x000167C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getSystemTime, ResponseMessageType_Uint64List(count=2)**********/
    void handleResponse0x000003F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getVersion, ResponseMessageType_Uint64_String**********/
    void handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********getRpcTable, ResponseMessageType_RpcTable**********/
    void handleResponse0x00004FA5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********getPublishTable, ResponseMessageType_PublishTable**********/
    void handleResponse0x000147A5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********tool_manager/addTool, ResponseMessageType_Bool**********/
    void handleResponse0x0000A22C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********tool_manager/deleteTool, ResponseMessageType_Bool**********/
    void handleResponse0x00010E4C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********tool_manager/updateTool, ResponseMessageType_Bool**********/
    void handleResponse0x0000C78C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********tool_manager/moveTool, ResponseMessageType_Bool**********/
    void handleResponse0x000085FC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********tool_manager/getToolInfoById, ResponseMessageType_Bool_ToolInfo**********/
    void handleResponse0x00009E34(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********tool_manager/getAllValidToolSummaryInfo, ResponseMessageType_ToolSummaryList**********/
    void handleResponse0x0001104F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********coordinate_manager/addUserCoord, ResponseMessageType_Bool**********/
    void handleResponse0x00016764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********coordinate_manager/deleteUserCoord, ResponseMessageType_Bool**********/
    void handleResponse0x0000BAF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********coordinate_manager/updateUserCoord, ResponseMessageType_Bool**********/
    void handleResponse0x0000EC14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********coordinate_manager/moveUserCoord, ResponseMessageType_Bool**********/
    void handleResponse0x0000E104(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********coordinate_manager/getUserCoordInfoById, ResponseMessageType_Bool_UserInfo**********/
    void handleResponse0x00004324(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********coordinate_manager/getAllValidUserCoordSummaryInfo, ResponseMessageType_UserSummaryList**********/
    void handleResponse0x0001838F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/r/addReg, ResponseMessageType_Bool**********/
    void handleResponse0x00004FF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/deleteReg, ResponseMessageType_Bool**********/
    void handleResponse0x000012F7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/updateReg, ResponseMessageType_Bool**********/
    void handleResponse0x00005757(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/getReg, ResponseMessageType_Bool_RRegData**********/
    void handleResponse0x0000EAB7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/moveReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000C877(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/getChangedList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x0000A904(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/r/getValidList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00008CE4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/mr/addReg, ResponseMessageType_Bool**********/
    void handleResponse0x000097E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/deleteReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000E5D7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/updateReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000E9B7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/getReg, ResponseMessageType_Bool_MrRegData**********/
    void handleResponse0x0000B507(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/moveReg, ResponseMessageType_Bool**********/
    void handleResponse0x00015BA7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/getChangedList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00001774(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/mr/getValidList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00015CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/sr/addReg, ResponseMessageType_Bool**********/
    void handleResponse0x000161E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/deleteReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000B817(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/updateReg, ResponseMessageType_Bool**********/
    void handleResponse0x000119F7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/getReg, ResponseMessageType_Bool_SrRegData**********/
    void handleResponse0x00017F07(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/moveReg, ResponseMessageType_Bool**********/
    void handleResponse0x00002127(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/getChangedList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00004834(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/sr/getValidList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00009854(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/pr/addReg, ResponseMessageType_Bool**********/
    void handleResponse0x000154E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/deleteReg, ResponseMessageType_Bool**********/
    void handleResponse0x00001097(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/updateReg, ResponseMessageType_Bool**********/
    void handleResponse0x00009EF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getReg, ResponseMessageType_Bool_PrRegData**********/
    void handleResponse0x00017207(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/moveReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000D7C7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getChangedList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x0000B454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getValidList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00009354(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/hr/addReg, ResponseMessageType_Bool**********/
    void handleResponse0x00016CE7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/deleteReg, ResponseMessageType_Bool**********/
    void handleResponse0x00003D17(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/updateReg, ResponseMessageType_Bool**********/
    void handleResponse0x0000CB77(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/getReg, ResponseMessageType_Bool_HrRegData**********/
    void handleResponse0x00000367(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/moveReg, ResponseMessageType_Bool**********/
    void handleResponse0x00014A87(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/getChangedList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00012974(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/hr/getValidList, ResponseMessageType_BaseRegSummaryList**********/
    void handleResponse0x00006B54(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/motion_control/stop, ResponseMessageType_Bool**********/
    void handleResponse0x00001E70(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/reset, ResponseMessageType_Bool**********/
    void handleResponse0x00001D14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doStepManualMove, ResponseMessageType_Bool**********/
    void handleResponse0x000085D5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doContinuousManualMove, ResponseMessageType_Bool**********/
    void handleResponse0x0000D3F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getJointsFeedBack, ResponseMessageType_Bool_Int32List(count = 9) **********/
    void handleResponse0x0000DFBB(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/motion_control/axis_group/setUserSoftLimit, ResponseMessageType_Bool**********/
    void handleResponse0x000114A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getUserSoftLimit, ResponseMessageType_Bool_JointLimit**********/
    void handleResponse0x0000C764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setManuSoftLimit, ResponseMessageType_Bool**********/
    void handleResponse0x000108E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getManuSoftLimit, ResponseMessageType_Bool_JointLimit**********/
    void handleResponse0x0000C244(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doGotoCartesianPointManualMove, ResponseMessageType_Bool**********/
    void handleResponse0x00010C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doGotoJointPointManualMove, ResponseMessageType_Bool**********/
    void handleResponse0x00008075(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doManualStop, ResponseMessageType_Bool**********/
    void handleResponse0x0000A9A0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setOrientationManualStep, ResponseMessageType_Uint64**********/
    void handleResponse0x00002940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getOrientationManualStep, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x00016D20(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setSingleZeroPointOffset, ResponseMessageType_Uint64**********/
    void handleResponse0x00012404(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/interpreter/start, ResponseMessageType_Bool**********/
    void handleResponse0x00006154(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/debug, ResponseMessageType_Bool**********/
    void handleResponse0x000102D7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/forward, ResponseMessageType_Bool**********/
    void handleResponse0x0000D974(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/backward, ResponseMessageType_Bool**********/
    void handleResponse0x00008E74(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/jump, ResponseMessageType_Bool**********/
    void handleResponse0x00015930(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/pause, ResponseMessageType_Bool**********/
    void handleResponse0x0000BA55(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/resume, ResponseMessageType_Bool**********/
    void handleResponse0x0000CF55(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/abort, ResponseMessageType_Bool**********/
    void handleResponse0x000086F4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/switchStep, ResponseMessageType_Uint64**********/
    void handleResponse0x000140F0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/setStartMode, ResponseMessageType_Uint64**********/
    void handleResponse0x00010225(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getStartMode, ResponseMessageType_Uint64_Int32**********/
    void handleResponse0x000092E5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/publish/addTopic, ResponseMessageType_Bool**********/
    void handleResponse0x000050E3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/deleteTopic, ResponseMessageType_Bool**********/
    void handleResponse0x00004403(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/addRegTopic, ResponseMessageType_Bool**********/
    void handleResponse0x000163A3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/addIoTopic, ResponseMessageType_Bool**********/
    void handleResponse0x000058F3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/deleteRegTopic, ResponseMessageType_Uint64**********/
    void handleResponse0x00010353(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/deleteIoTopic, ResponseMessageType_Uint64**********/
    void handleResponse0x0000DD03(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********rpc/motion_control/setGlobalVelRatio, ResponseMessageType_Uint64**********/
    void handleResponse0x000005EF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getGlobalVelRatio, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x0001578F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/setGlobalAccRatio, ResponseMessageType_Uint64**********/
    void handleResponse0x0000271F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getGlobalAccRatio, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x00016D9F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setHardLimit, ResponseMessageType_Uint64**********/
    void handleResponse0x0000C454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getHardLimit, ResponseMessageType_Uint64_JointLimit **********/
    void handleResponse0x00013394(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getCoordinate, ResponseMessageType_Uint64_Int32List(count = 2)**********/
    void handleResponse0x00008595(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setCoordinate, ResponseMessageType_Uint64**********/
    void handleResponse0x0000A845(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    //"/rpc/motion_control/axis_group/getUserCoordId"
    void handleResponse0x00005BB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    //"/rpc/motion_control/axis_group/setUserCoordId"
    void handleResponse0x00005CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);    
    /********rpc/motion_control/axis_group/getTool, ResponseMessageType_Uint64_Int32**********/
    void handleResponse0x0001354C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setTool, ResponseMessageType_Uint64**********/
    void handleResponse0x0001581C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/convertCartToJoint, ResponseMessageType_Uint64_DoubleList(count = 9)**********/
    void handleResponse0x00010FD4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/convertJointToCart, ResponseMessageType_Uint64_DoubleList(count = 6)**********/
    void handleResponse0x0000B6D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********rpc/motion_control/axis_group/ignoreLostZeroError, ResponseMessageType_Uint64**********/
    void handleResponse0x00014952(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
        /********rpc/motion_control/axis_group/setAllZeroPointOffsets, ResponseMessageType_Uint64**********/
    void handleResponse0x00008AB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getAllZeroPointOffsets, ResponseMessageType_Uint64_DoubleList(count=9)**********/
    void handleResponse0x00012353(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getAllZeroErrorMaskStatus, ResponseMessageType_Uint64_Int32List(count=9)**********/
    void handleResponse0x0000C183(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/saveAllZeroPointOffsets, ResponseMessageType_Uint64**********/
    void handleResponse0x000171D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setSingleZeroPointStatus, ResponseMessageType_Uint64**********/
    void handleResponse0x00010E43(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    //"/rpc/motion_control/axis_group/getAllZeroPointStatus"
    void handleResponse0x000102F3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateAllZeroPointOffsets, ResponseMessageType_Uint64**********/	
    void handleResponse0x00011B03(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateSingleZeroPointOffset, ResponseMessageType_Uint64**********/
    void handleResponse0x000131D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateZeroPointOffsets, ResponseMessageType_Uint64**********/
    void handleResponse0x00005AE3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/isReferencePointExist, ResponseMessageType_Uint64_Bool**********/
    void handleResponse0x0000D344(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/deleteReferencePoint, ResponseMessageType_Uint64**********/
    void handleResponse0x00008744(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/saveReferencePoint, ResponseMessageType_Uint64**********/
    void handleResponse0x00006744(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets, ResponseMessageType_Uint64**********/
    void handleResponse0x0000E913(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset, ResponseMessageType_Uint64**********/
    void handleResponse0x00004754(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets, ResponseMessageType_Uint64**********/
    void handleResponse0x00007EC3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getAxisGroupInfoList, ResponseMessageType_Uint64_AxisGroupInfoList(count = 8)**********/
    void handleResponse0x00010F54(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getUserSoftLimitWithUnit, ResponseMessageType_Uint64_JointLimitWithUnit**********/
    void handleResponse0x00008ED4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getManuSoftLimitWithUnit, ResponseMessageType_Uint64_JointLimitWithUnit**********/
    void handleResponse0x000124E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getHardLimitWithUnit, ResponseMessageType_Uint64_JointLimitWithUnit**********/
    void handleResponse0x000092B4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setRotateManualStep, ResponseMessageType_Uint64**********/
    void handleResponse0x00005290(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getRotateManualStep, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x00003000(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setPrismaticManualStep, ResponseMessageType_Uint64**********/
    void handleResponse0x0000B640(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getPrismaticManualStep, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x0000FCE0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setCartesianManualStep, ResponseMessageType_Uint64**********/
    void handleResponse0x0000A420(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getCartesianManualStep, ResponseMessageType_Uint64_Double**********/
    void handleResponse0x0000EAC0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********UserOpMode, MessageType_Int32**********/  
    void handlePublishElement0x00015255(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********RunningStatus, MessageType_Int32**********/
    void handlePublishElement0x00001F33(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********InterpreterStatus, MessageType_Int32**********/
    void handlePublishElement0x00003203(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********RobotStatus, MessageType_Int32**********/
    void handlePublishElement0x00012943(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********CtrlStatus, MessageType_Int32**********/
    void handlePublishElement0x0000E8E3(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********ServoStatus, MessageType_Int32**********/
    void handlePublishElement0x00002053(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********SafetyAlarm, MessageType_Int32**********/
    void handlePublishElement0x0000D0AD(Comm_Publish& package, int element_index, TpPublishElement& list_element);

    /********"MessageType_Int32_Int32List(count=9)",**********/
    void handlePublishElement0x000161F3(Comm_Publish& package, int element_index, TpPublishElement& list_element);

	/********publish/motion_control/axis_group/feedback/tcp_world_cartesian, MessageType_Int32_DoubleList(count=6)**********/
    void handlePublishElement0x00009D8E(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/axis_group/feedback/tcp_base_cartesian, MessageType_Int32_DoubleList(count=6)**********/
    void handlePublishElement0x00002D5E(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/axis_group/feedback/tcp_current_cartesian, MessageType_Int32_DoubleList(count=6)**********/
    void handlePublishElement0x0000352E(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/axis_group/current_coordinate, MessageType_Int32List(count=3)**********/
    void handlePublishElement0x00012C55(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/axis_group/current_tool, MessageType_Int32List(count=2)**********/
    void handlePublishElement0x00004BEC(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/global_vel_ratio, MessageType_Double**********/
    void handlePublishElement0x00012A4F(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/motion_control/global_acc_ratio, MessageType_Double**********/
    void handlePublishElement0x0001517F(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/interpreter/program_status, MessageType_String_Int32**********/
    void handlePublishElement0x00001AF3(Comm_Publish& package, int element_index, TpPublishElement& list_element);
	/********publish/interpreter/tp_program_status, MessageType_StringList(count=2)**********/
    void handlePublishElement0x000042B3(Comm_Publish& package, int element_index, TpPublishElement& list_element);

   /********rpc/controller/addIoTopic, ResponseMessageType_Bool**********/
    void handleResponse0x0000B823(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/io_mapping/getDIByBit, ResponseMessageType_Bool_Int32**********/
    void handleResponse0x000050B4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/setDIByBit, ResponseMessageType_Bool**********/
    void handleResponse0x00011754(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/getDOByBit, ResponseMessageType_Bool_Int32**********/
    void handleResponse0x00013074(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/setDOByBit, ResponseMessageType_Bool**********/
    void handleResponse0x00007074(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
 

   /********rpc/io_mapping/getRIByBit, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x00000684(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/setRIByBit, ResponseMessageType_Uint64**********/
    void handleResponse0x0000CD24(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/getROByBit, ResponseMessageType_Uint64_Int32**********/
    void handleResponse0x00005BD4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/setROByBit, ResponseMessageType_Uint64**********/
    void handleResponse0x00012274(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/syncFileIoStatus, ResponseMessageType_Uint64**********/
    void handleResponse0x0000BA73(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io_mapping/syncFileIoMapping, ResponseMessageType_Uint64**********/
    void handleResponse0x0000C2A7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/device_manager/getDeviceList , ResponseMessageType_DeviceInfoList**********/
    void handleResponse0x0000C1E0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/device_manager/get_FRP8A_IoDeviceInfo, ResponseMessageType_Uint64_IoDeviceInfo**********/
    void handleResponse0x00006BAF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/device_manager/getModbusIoDeviceInfo, ResponseMessageType_Uint64_IoDeviceInfo**********/
    void handleResponse0x0001421F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/device_manager/getIoDeviceInfoList, ResponseMessageType_Uint64_IoDeviceInfoList**********/
    void handleResponse0x000024A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/program_launching/setMethod, ResponseMessageType_Uint64**********/
    void handleResponse0x00011544(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/program_launching/getMethod, ResponseMessageType_Uint64_Int32**********/
    void handleResponse0x00010944(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/program_launching/syncFileMacroConfig, ResponseMessageType_Uint64**********/
    void handleResponse0x00016B27(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/modbus/newServer, ResponseMessageType_Uint64**********/
    void handleResponse0x0000D1B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/deleteServer, ResponseMessageType_Uint64**********/
    void handleResponse0x00006C22(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/saveServerConfig, ResponseMessageType_Uint64**********/
    void handleResponse0x000050E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/newClient, ResponseMessageType_Uint64**********/
    void handleResponse0x00009F84(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/deleteClient, ResponseMessageType_Uint64**********/
    void handleResponse0x00014CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/saveClientConfig, ResponseMessageType_Uint64**********/
    void handleResponse0x00002B57(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
	/********rpc/modbus/getServerConfig, ResponseMessageType_Uint64_ModbusServerConfig**********/
    void handleResponse0x00016947(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
	/********rpc/modbus/getClientConfig, ResponseMessageType_Uint64_ModbusClientConfig**********/
    void handleResponse0x0000FC17(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/modbus/getConnectionStatus, ResponseMessageType_Uint64_Bool**********/
    void handleResponse0x0000E973(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/modbus/setConnectionStatus, ResponseMessageType_Uint64**********/
    void handleResponse0x00010C63(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    fst_ip::LocalIP local_ip_;
    TpCommManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    bool is_received_;
    bool is_running_;
    std::string req_resp_ip_;
    std::string publish_ip_;

    // component parameters
    int cycle_time_;//ms
    int recv_buffer_size_;
    int send_buffer_size_;

    // runtime variables
    int req_resp_socket_;
    int publish_socket_;
    int req_resp_endpoint_id_;
    int publish_endpoint_id_;
    struct nn_pollfd poll_fd_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    fst_base::ThreadHelp thread_ptr_;

    // runtime table
    typedef struct
    {
        std::string path;
        unsigned int hash;
        std::string request_type;
        std::string response_type;
        HandleRequestFuncPtr request_func_ptr;
        HandleResponseFuncPtr response_func_ptr;
        Comm_Authority authority;
    }RpcService;
    std::vector<RpcService> rpc_table_;
    std::vector<RpcService> rpc_quick_search_table_[QUICK_SEARCH_TABLE_SIZE];

    typedef struct
    {
        std::string element_path;
        unsigned int hash;
        std::string element_type;
        HandlePublishElementFuncPtr publish_element_func_ptr;
        Comm_Authority authority;
    }PublishService;
    std::vector<PublishService> publish_element_table_;
    std::vector<PublishService> publish_element_quick_search_table_[QUICK_SEARCH_TABLE_SIZE];

    // runtime list & list mutex
    std::mutex request_list_mutex_;
    std::mutex response_list_mutex_;
    std::mutex publish_list_mutex_;
    std::mutex reg_publish_list_mutex_;
    std::mutex io_publish_list_mutex_;
    std::list<TpRequestResponse>  request_list_;
    std::vector<TpRequestResponse>  response_list_;
    std::vector<TpPublish>  publish_list_;
    std::vector<TpPublish>  reg_publish_list_;
    std::vector<TpPublish>  io_publish_list_;
};
}
#endif

void tpCommRoutineThreadFunc(void* arg);