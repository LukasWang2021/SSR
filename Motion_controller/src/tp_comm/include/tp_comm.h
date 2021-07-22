#ifndef TP_COMM_H
#define TP_COMM_H

/**
 * @file tp_comm.h
 * @brief The file includes the class for handling socket communication.
 * @author zhengyu.shen
 */

#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <sys/time.h>
#include <nanomsg/nn.h>

#include "common_error_code.h"
#include "protoc.h"
#include "yaml_help.h"
#include "thread_help.h"
#include "local_ip.h"

#include "tp_comm_manager_config.h"
#include "log_manager_producer.h"
#include "error_queue.h"

/**
 * @brief user_space includes all user-defined implementations.
 */
namespace user_space
{
/**
 * @brief Stores the information of a publishing element.
 */
typedef struct
{
    unsigned int hash;  /**< The hash code of the pulishing element.*/
    void* data_ptr;     /**< Pointer of the data to be published.*/
}TpPublishElement;
/**
 * @brief Stores the configuration of a topic to be published.
 */
typedef struct
{
    unsigned int hash;                              /**< The hash code of the topic.*/
    std::vector<TpPublishElement> element_list_;    /**< The list of elements to in the topic.*/
    Comm_Publish package;                           /**< Proto buffer type of the topic.*/
    int interval_min;                               /**< The minimum interval between two contiguous pushlishments, unit in ms.*/
    int interval_max;                               /**< The maximum interval between two contiguous pushlishments, unit in ms.*/
    bool is_element_changed;                        /**< The flag to show if the value of elements to be published have been changed.*/
    struct timeval last_publish_time;               /**< Record the system time for the last publishment.*/
}TpPublish;
/**
 * @brief Stores the information of an RPC.
 */
typedef struct
{
    unsigned int hash;      /**< The hash code of the RPC.*/
    void* request_data_ptr; /**< Pointer to the proto buffer type of the request.*/
    void* response_data_ptr;/**< Pointer to the proto buffer type of the response.*/
}TpRequestResponse;
/**
 * @brief Stores the information of an event.
 */
typedef struct
{
    int type;                           /**< The event type.*/
    int to_id;                          /**< Id of the expected event receiver.*/
	unsigned long long int event_data;  /**< Data of the event*/
}TpEventMsg;
/**
 * @brief TpComm is the object to handle socket communication.
 * @details The communication is based on TCP/IP protocal in data-link layer and handles application demands by proto-buffer technique.\n
 *          The object provides three communication channels: RPC, publish-subscribe and event.
 */
class TpComm
{
public:
    /**
     * @brief Constructor of the class.
     */    
    TpComm();
    /**
     * @brief Destructor of the class.
     */    
    ~TpComm();
    /**
     * @brief Initialize internal data.
     * @retval SUCCESS Operation succeed.
     * @retval TP_COMM_LOAD_PARAM_FAILED Failed to load configuration of the module.
     * @retval TP_COMM_INIT_OBJECT_FAILED Failed to create sockets.
     */
    unsigned long long int init();

    /**
     * @brief Rountine thread function of the module.
     * @details Handling request, response, publishing and event in the function.
     * @return void
     */
    void tpCommThreadFunc();
    /**
     * @brief Check if the thread function is running.
     * @return true The thread is running.
     * @return false The thread is not running.
     */
    bool isRunning();
    /**
     * @brief Pop request from the request list.
     * @details The API is called by other module to handling some requests.
     * @param [out] task Request/Response package.
     * @retval true Get a request.
     * @retval false The list is empty, no request exists.
     */
    bool popTaskFromRequestList(TpRequestResponse* task);
    /**
     * @brief Check if the RPC is correct in communication level.
     * @details It is necessary to call the API to make sure the data is correct before handling any RPC in application level.\n
     * @param [in] response_data_ptr Pointer of the response data of the RPC.
     * @return Value 0 presents the RPC is correct in communication level. Otherwise, there is something wrong for the RPC data.
     */    
    uint64_t getResponseSucceed(void* response_data_ptr);
    /**
     * @brief Push response to the response list.
     * @details The API is called by other module after finish handling some requests.
     * @param [out] package Request/Response package.
     * @return void
     */    
    void pushTaskToResponseList(TpRequestResponse& package);
    /**
     * @brief Create a topic object with empty element.
     * @param [in] topic_hash Topic hash code.
     * @param [in] interval_min The minimum interval between two contiguous pushlishments, unit in ms.
     * @param [in] interval_max The maximum interval between two contiguous pushlishments, unit in ms.
     * @return The configuration of the topic.
     */     
    TpPublish generateTpPublishTask(unsigned int topic_hash, int interval_min, int interval_max);
    /**
     * @brief Add an element into a topic.
     * @param [in] task The configuration of a topic.
     * @param [in] element_hash Hash code of an element.
     * @param [in] element_data_ptr Pointer of the data of the element.
     * @return void
     */     
    void addTpPublishElement(TpPublish& task, unsigned int element_hash, void* element_data_ptr);
    /**
     * @brief Check if some topic has already been exist.
     * @param [in] topic_hash Hash code of a topic.
     * @retval true The topic with given hash code has already been exist.
     * @retval false The topic with given hash code is not exist.
     */ 
    bool isTopicExisted(unsigned int topic_hash);
    /**
     * @brief Add a topic to the routine thread of the module.
     * @param [in] package The configuration of a topic.
     * @return void.
     */      
    void pushTaskToPublishList(TpPublish& package);
    /**
     * @brief Delete a topic from the routine thread of the module.
     * @param [in] topic_hash Hash code of a topic.
     * @return A list of elements in the topic that has been deleted.
     */    
    std::vector<TpPublishElement> eraseTaskFromPublishList(unsigned int &topic_hash);
    /**
     * @brief Lock mutex of the element updating list for publish.
     * @return void
     */  
    void lockPublishMutex();
    /**
     * @brief Unlock mutex of the element updating list for publish.
     * @return void
     */     
    void unlockPublishMutex();
    /**
     * @brief Send a event to some receiver.
     * @param [in] event The configuration of a event.
     * @return void
     */ 
	void sendEvent(TpEventMsg& event);

private:
    enum {HASH_BYTE_SIZE = 4,};             /**< Byte size of hash code.*/
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};  /**< Size of quick search table.*/

    typedef void (TpComm::*HandleRequestFuncPtr)(int);
    typedef void (TpComm::*HandleResponseFuncPtr)(std::vector<TpRequestResponse>::iterator&, int&);
    typedef void (TpComm::*HandlePublishElementFuncPtr)(Comm_Publish&, int, TpPublishElement&);

    unsigned long long int open();
    void close();
	
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
	void handleSendEventList(void);

    long long computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val);
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
    bool encodeEventPackage(const pb_field_t fields[], void* event_data_ptr, int& send_buffer_size);

    void handleRequestNonexistentHash(int hash, int recv_bytes);
    void handleResponseNonexistentHash(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/publish/addTopic, RequestMessageType_Topic**********/	
    void handleRequest0x000050E3(int recv_bytes);
    /********rpc/publish/deleteTopic, RequestMessageType_Uint32**********/	
    void handleRequest0x00004403(int recv_bytes);
        
    /********rpc/tp_comm/getRpcTable, RequestMessageType_Void**********/	
    void handleRequest0x00004FA5(int recv_bytes);
    /********rpc/tp_comm/getPublishTable, RequestMessageType_Void**********/	
    void handleRequest0x000147A5(int recv_bytes);
        
    /********rpc/file_manager/readFile, RequestMessageType_String**********/	
    void handleRequest0x0000A545(int recv_bytes);
    /********rpc/file_manager/writeFile, RequestMessageType_String_Bytes**********/	
    void handleRequest0x00010D95(int recv_bytes);
        
    /********rpc/controller/getVersion, RequestMessageType_Void**********/	
    void handleRequest0x000093EE(int recv_bytes);
    /********rpc/controller/setSystemTime, RequestMessageType_Uint64**********/	
    void handleRequest0x000167C5(int recv_bytes);
    /********rpc/controller/getSystemTime, RequestMessageType_Void**********/	
    void handleRequest0x000003F5(int recv_bytes);
    /********rpc/controller/setWorkMode, RequestMessageType_Uint32**********/	
    void handleRequest0x00006825(int recv_bytes);
    /********rpc/controller/getWorkMode, RequestMessageType_Void**********/	
    void handleRequest0x00003325(int recv_bytes);
    /********rpc/controller/setControlMode, RequestMessageType_Uint32**********/	
    void handleRequest0x0000B555(int recv_bytes);
    /********rpc/controller/getControlMode, RequestMessageType_Void**********/	
    void handleRequest0x0000B695(int recv_bytes);

        
    /********rpc/axis/mcPower, RequestMessageType_Int32_Bool**********/	
    void handleRequest0x000053E2(int recv_bytes);
    /********rpc/axis/mcReset, RequestMessageType_Int32**********/	
    void handleRequest0x000180C4(int recv_bytes);
    /********rpc/axis/mcStop, RequestMessageType_Int32**********/	
    void handleRequest0x00002820(int recv_bytes);
    /********rpc/axis/mcHalt, RequestMessageType_Int32**********/	
    void handleRequest0x00004BB4(int recv_bytes);
    /********rpc/axis/mcSetPosition, RequestMessageType_Int32_Double**********/	
    void handleRequest0x0001798E(int recv_bytes);
    /********rpc/axis/mcReadParameter, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00016BF2(int recv_bytes);
    /********rpc/axis/mcWriteParameter, RequestMessageType_Int32List(count=3)**********/	
    void handleRequest0x00005732(int recv_bytes);
    /********rpc/axis/mcMoveAbsolute, RequestMessageType_Int32_DoubleList(count=5)**********/	
    void handleRequest0x000051F5(int recv_bytes);
    /********rpc/axis/mcMoveVelocity, RequestMessageType_Int32List_DoubleList(count=2,count=4)**********/	
    void handleRequest0x00016CF9(int recv_bytes);
    /********rpc/axis/mcReadActualPosition, RequestMessageType_Int32**********/	
    void handleRequest0x000012BE(int recv_bytes);
    /********rpc/axis/mcReadActualVelocity, RequestMessageType_Int32**********/	
    void handleRequest0x00002EA9(int recv_bytes);
    /********rpc/axis/mcReadActualTorque, RequestMessageType_Int32**********/	
    void handleRequest0x00014265(int recv_bytes);
    /********rpc/axis/mcReadAxisInfo, RequestMessageType_Int32**********/	
    void handleRequest0x0000314F(int recv_bytes);
    /********rpc/axis/mcReadStatus, RequestMessageType_Int32**********/	
    void handleRequest0x00003E53(int recv_bytes);
    /********rpc/axis/mcReadAxisError, RequestMessageType_Int32**********/	
    void handleRequest0x000063C2(int recv_bytes);
    /********rpc/axis/mcReadAxisErrorHistory, RequestMessageType_Int32**********/	
    void handleRequest0x00018469(int recv_bytes);
    /********rpc/axis/mcMoveRelative, RequestMessageType_Int32_DoubleList(count=5)**********/	
    void handleRequest0x0000CC85(int recv_bytes);
    /********rpc/axis/mcHome, RequestMessageType_Int32**********/	
    void handleRequest0x000059B5(int recv_bytes);
    /********rpc/axis/rtmAbortHoming, RequestMessageType_Int32**********/	
    void handleRequest0x0000E4B7(int recv_bytes);
    /********rpc/axis/rtmReadAxisFdbPdoPtr, RequestMessageType_Int32**********/	
    void handleRequest0x0000A632(int recv_bytes);
    /********rpc/axis/rtmResetEncoder, RequestMessageType_Int32**********/
    void handleRequest0x00000BA2(int recv_bytes);
        
    /********rpc/group/mcGroupReset, RequestMessageType_Int32**********/	
    void handleRequest0x00016FF4(int recv_bytes);
    /********rpc/group/mcGroupEnable, RequestMessageType_Int32**********/	
    void handleRequest0x00003615(int recv_bytes);
    /********rpc/group/mcGroupDisable, RequestMessageType_Int32**********/	
    void handleRequest0x0000D185(int recv_bytes);
    /********rpc/group/mcGroupReadError, RequestMessageType_Int32**********/	
    void handleRequest0x00004BE2(int recv_bytes);
    /********rpc/group/mcGroupReadStatus, RequestMessageType_Int32**********/	
    void handleRequest0x00002A83(int recv_bytes);
    /********rpc/group/resetAllEncoder, RequestMessageType_Int32**********/	
    void handleRequest0x000019D2(int recv_bytes);

    /********rpc/servo_sampling/setSamplingConfiguration, RequestMessageType_Int32_Uint32List(count=2)**********/	
    void handleRequest0x0000845E(int recv_bytes);
    /********rpc/servo_sampling/getSamplingConfiguration, RequestMessageType_Int32**********/	
    void handleRequest0x000106EE(int recv_bytes);
    /********rpc/servo_sampling/activateSamplingConfiguration, RequestMessageType_Int32**********/	
    void handleRequest0x0000CDDE(int recv_bytes);
    /********rpc/servo_sampling/setSamplingSync, RequestMessageType_Int32_Uint32**********/	
    void handleRequest0x00003743(int recv_bytes);
    /********rpc/servo_sampling/getSamplingSync, RequestMessageType_Int32**********/	
    void handleRequest0x00006343(int recv_bytes);
    /********rpc/servo_sampling/setSamplingChannel, RequestMessageType_Int32List(count=4)**********/	
    void handleRequest0x0000BACC(int recv_bytes);
    /********rpc/servo_sampling/getSamplingChannel, RequestMessageType_Int32**********/	
    void handleRequest0x0000556C(int recv_bytes);
    /********rpc/servo_sampling/saveSamplingBufferData, RequestMessageType_Int32_String**********/	
    void handleRequest0x00004E41(int recv_bytes);
        
    /********rpc/servo1001/servo/shutDown, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000863E(int recv_bytes);
    /********rpc/servo1001/servo/switchOn, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000E5CE(int recv_bytes);
    /********rpc/servo1001/servo/disableVoltage, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00004755(int recv_bytes);
    /********rpc/servo1001/servo/enableOperation, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000313E(int recv_bytes);
    /********rpc/servo1001/servo/switchOnAndEnableOperation, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000177CE(int recv_bytes);
    /********rpc/servo1001/servo/disableOperation, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000026AE(int recv_bytes);
    /********rpc/servo1001/servo/quickStop, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00000580(int recv_bytes);
    /********rpc/servo1001/servo/resetFault, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00010584(int recv_bytes);
    /********rpc/servo1001/servo/transCommState, RequestMessageType_Int32List_CoreCommState(count=2)**********/	
    void handleRequest0x000153C5(int recv_bytes);
    /********rpc/servo1001/servo/readParameter, RequestMessageType_Int32List(count=3)**********/	
    void handleRequest0x00006892(int recv_bytes);
    /********rpc/servo1001/servo/writeParameter, RequestMessageType_Int32List(count=4)**********/	
    void handleRequest0x00007C32(int recv_bytes);
    /********rpc/servo1001/servo/moveVelocity, RequestMessageType_Int32List(count=7)**********/	
    void handleRequest0x000164D9(int recv_bytes);
    /********rpc/servo1001/servo/moveAbsolute, RequestMessageType_Int32List_Int64(count=6)**********/	
    void handleRequest0x00004DD5(int recv_bytes);
    /********rpc/servo1001/servo/triggerUploadParameters, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000020B3(int recv_bytes);
    /********rpc/servo1001/servo/uploadParameters, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000E003(int recv_bytes);
    /********rpc/servo1001/servo/triggerDownloadParameters, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00011C53(int recv_bytes);
    /********rpc/servo1001/servo/downloadParameters, RequestMessageType_Int32List_Int32List(count=2,count=512)**********/	
    void handleRequest0x00017063(int recv_bytes);
    /********rpc/servo1001/servo/isAsyncServiceFinish, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000043B8(int recv_bytes);
    /********rpc/servo1001/servo/getCommState, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000F485(int recv_bytes);
    /********rpc/servo1001/servo/getServoState, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000032F5(int recv_bytes);
    /********rpc/servo1001/servo/moveRelative, RequestMessageType_Int32_DoubleList(count=5)**********/	
    void handleRequest0x000172C5(int recv_bytes);
    /********rpc/servo1001/servo/resetEncoder, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000EFE2(int recv_bytes);
    /********rpc/servo1001/servo/goHome, RequestMessageType_Int32**********/	
    void handleRequest0x00013BB5(int recv_bytes);
    /********rpc/servo1001/servo/abortHoming, RequestMessageType_Int32**********/	
    void handleRequest0x00015AB7(int recv_bytes);
        
    /********rpc/servo1001/cpu/getVersion, RequestMessageType_Int32**********/	
    void handleRequest0x0001192E(int recv_bytes);
    /********rpc/servo1001/cpu/setCtrlPdoSync, RequestMessageType_Int32List_Uint32**********/	
    void handleRequest0x00005123(int recv_bytes);
    /********rpc/servo1001/cpu/getCtrlPdoSync, RequestMessageType_Int32List**********/	
    void handleRequest0x00005463(int recv_bytes);
    /********rpc/servo1001/cpu/setSamplingSync, RequestMessageType_Int32_Uint32**********/	
    void handleRequest0x00004023(int recv_bytes);
    /********rpc/servo1001/cpu/getSamplingSync, RequestMessageType_Int32**********/	
    void handleRequest0x00006C23(int recv_bytes);
    /********rpc/servo1001/cpu/setSamplingInterval, RequestMessageType_Int32_Uint32**********/	
    void handleRequest0x00003EEC(int recv_bytes);
    /********rpc/servo1001/cpu/getSamplingInterval, RequestMessageType_Int32**********/	
    void handleRequest0x00001C2C(int recv_bytes);
    /********rpc/servo1001/cpu/setSamplingMaxTimes, RequestMessageType_Int32_Uint32**********/	
    void handleRequest0x000110A3(int recv_bytes);
    /********rpc/servo1001/cpu/getSamplingMaxTimes, RequestMessageType_Int32**********/	
    void handleRequest0x00013363(int recv_bytes);
    /********rpc/servo1001/cpu/setSamplingChannel, RequestMessageType_Int32_Uint32List(count=3)**********/	
    void handleRequest0x00008E5C(int recv_bytes);
    /********rpc/servo1001/cpu/getSamplingChannel, RequestMessageType_Int32**********/	
    void handleRequest0x0000FD9C(int recv_bytes);
    /********rpc/servo1001/cpu/activateSamplingConfiguration, RequestMessageType_Int32**********/	
    void handleRequest0x0000939E(int recv_bytes);
    /********rpc/servo1001/cpu/saveSamplingBufferData, RequestMessageType_Int32_String**********/	
    void handleRequest0x00015621(int recv_bytes);
    /********rpc/servo1001/servo/getServoCommInfo, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x0000BF1F(int recv_bytes);
    /********rpc/servo1001/cpu/getServoCpuCommInfo, RequestMessageType_Int32**********/	
    void handleRequest0x0000FE5F(int recv_bytes);
    /********rpc/servo1001/servo/getServoDefinedInfo, RequestMessageType_Int32_Int32List(count=9)**********/	
    void handleRequest0x0000C87F(int recv_bytes);	
    
    /********rpc/io/readDI, RequestMessageType_Int32**********/	
    void handleRequest0x000185A9(int recv_bytes);
    /********rpc/io/readDO, RequestMessageType_Int32**********/	
    void handleRequest0x000185AF(int recv_bytes);
    /********rpc/io/writeDO, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00000C1F(int recv_bytes);

    /********rpc/tool_manager/addTool, RequestMessageType_ToolInfo**********/	
    void handleRequest0x0000A22C(int recv_bytes);
    /********rpc/tool_manager/deleteTool, RequestMessageType_Int32**********/	
    void handleRequest0x00010E4C(int recv_bytes);
    /********rpc/tool_manager/updateTool, RequestMessageType_ToolInfo**********/	
    void handleRequest0x0000C78C(int recv_bytes);
    /********rpc/tool_manager/moveTool, RequestMessageType_Int32List(count = 2) **********/	
    void handleRequest0x000085FC(int recv_bytes);
    /********rpc/tool_manager/getToolInfoById, RequestMessageType_Int32**********/	
    void handleRequest0x00009E34(int recv_bytes);
    /********rpc/tool_manager/getAllValidToolSummaryInfo, RequestMessageType_Void**********/	
    void handleRequest0x0001104F(int recv_bytes);

    /********rpc/coordinate_manager/addUserCoord, RequestMessageType_UserCoordInfo**********/	
    void handleRequest0x00016764(int recv_bytes);
    /********rpc/coordinate_manager/deleteUserCoord, RequestMessageType_Int32**********/	
    void handleRequest0x0000BAF4(int recv_bytes);
    /********rpc/coordinate_manager/updateUserCoord, RequestMessageType_UserCoordInfo**********/	
    void handleRequest0x0000EC14(int recv_bytes);
    /********rpc/coordinate_manager/moveUserCoord, RequestMessageType_Int32List(count = 2) **********/	
    void handleRequest0x0000E104(int recv_bytes);
    /********rpc/coordinate_manager/getUserCoordInfoById, RequestMessageType_Int32**********/	
    void handleRequest0x00004324(int recv_bytes);
    /********rpc/coordinate_manager/getAllValidUserCoordSummaryInfo, RequestMessageType_Void**********/	
    void handleRequest0x0001838F(int recv_bytes);


    /********rpc/motion_control/setGlobalVelRatio, RequestMessageType_Double**********/	
    void handleRequest0x000005EF(int recv_bytes);
    /********rpc/motion_control/getGlobalVelRatio, RequestMessageType_Void**********/	
    void handleRequest0x0001578F(int recv_bytes);
    /********rpc/motion_control/setGlobalAccRatio, RequestMessageType_Double**********/	
    void handleRequest0x0000271F(int recv_bytes);
    /********rpc/motion_control/getGlobalAccRatio, RequestMessageType_Void**********/	
    void handleRequest0x00016D9F(int recv_bytes);
    /********rpc/motion_control/axis_group/doStepManualMove, RequestMessageType_Int32_Int32List(count = 9) **********/	
    void handleRequest0x000085D5(int recv_bytes);
    /********rpc/motion_control/axis_group/doContinuousManualMove, RequestMessageType_Int32_Int32List(count = 9) **********/	
    void handleRequest0x0000D3F5(int recv_bytes);
    /********rpc/motion_control/axis_group/doGotoCartesianPointManualMove, RequestMessageType_Int32_UFTF_PoseAndPosture**********/	
    void handleRequest0x00010C05(int recv_bytes);
    /********rpc/motion_control/axis_group/doGotoJointPointManualMove, RequestMessageType_Int32_DoubleList(count = 9) **********/	
    void handleRequest0x00008075(int recv_bytes);
    /********rpc/motion_control/axis_group/doManualStop, RequestMessageType_Int32**********/	
    void handleRequest0x0000A9A0(int recv_bytes);
    /********rpc/motion_control/axis_group/getJointsFeedBack, RequestMessageType_Int32**********/	
    void handleRequest0x0000DFBB(int recv_bytes);
    /********rpc/motion_control/axis_group/setUserSoftLimit, RequestMessageType_Int32_JointLimit **********/	
    void handleRequest0x000114A4(int recv_bytes);
    /********rpc/motion_control/axis_group/getUserSoftLimit, RequestMessageType_Int32**********/	
    void handleRequest0x0000C764(int recv_bytes);
    /********rpc/motion_control/axis_group/setManuSoftLimit, RequestMessageType_Int32_JointLimit **********/	
    void handleRequest0x000108E4(int recv_bytes);
    /********rpc/motion_control/axis_group/getManuSoftLimit, RequestMessageType_Int32**********/	
    void handleRequest0x0000C244(int recv_bytes);
    /********rpc/motion_control/axis_group/setHardLimit, RequestMessageType_Int32_JointLimit **********/	
    void handleRequest0x0000C454(int recv_bytes);
    /********rpc/motion_control/axis_group/getHardLimit, RequestMessageType_Int32**********/	
    void handleRequest0x00013394(int recv_bytes);
    /********rpc/motion_control/axis_group/setCoordinate, RequestMessageType_Int32List(count = 2) **********/	
    void handleRequest0x0000A845(int recv_bytes);
    /********rpc/motion_control/axis_group/getCoordinate, RequestMessageType_Int32**********/	
    void handleRequest0x00008595(int recv_bytes);
    /********rpc/motion_control/axis_group/setUserCoordId, RequestMessageType_Int32List(count = 2) **********/	
    void handleRequest0x00005CF4(int recv_bytes);
    /********rpc/motion_control/axis_group/getUserCoordId, RequestMessageType_Int32**********/	
    void handleRequest0x00005BB4(int recv_bytes);
    /********rpc/motion_control/axis_group/setTool, RequestMessageType_Int32List(count = 2) **********/	
    void handleRequest0x0001581C(int recv_bytes);
    /********rpc/motion_control/axis_group/getTool, RequestMessageType_Int32**********/	
    void handleRequest0x0001354C(int recv_bytes);
    /********rpc/motion_control/axis_group/convertCartToJoint, RequestMessageType_Int32_UFTF_PoseAndPosture**********/	
    void handleRequest0x00010FD4(int recv_bytes);
    /********rpc/motion_control/axis_group/convertJointToCart, RequestMessageType_Int32List_DoubleList(Int32List_count=3,DoubleList_count = 9)**********/	
    void handleRequest0x0000B6D4(int recv_bytes);
    /********rpc/motion_control/axis_group/ignoreLostZeroError, RequestMessageType_Int32**********/	
    void handleRequest0x00014952(int recv_bytes);
    /********rpc/motion_control/axis_group/setSingleZeroPointOffset, RequestMessageType_Int32List_Double(count=2)**********/	
    void handleRequest0x00012404(int recv_bytes);
    /********rpc/motion_control/axis_group/setAllZeroPointOffsets, RequestMessageType_Int32_DoubleList(count=9)**********/	
    void handleRequest0x00011853(int recv_bytes);
    /********rpc/motion_control/axis_group/getAllZeroPointOffsets, RequestMessageType_Int32**********/	
    void handleRequest0x00012353(int recv_bytes);
    /********rpc/motion_control/axis_group/getAllZeroErrorMaskStatus, RequestMessageType_Int32**********/	
    void handleRequest0x0000C183(int recv_bytes);
    /********rpc/motion_control/axis_group/setSingleZeroPointStatus, RequestMessageType_Int32List(count=3)**********/	
    void handleRequest0x00010E43(int recv_bytes);
    /********rpc/motion_control/axis_group/getAllZeroPointStatus, RequestMessageType_Int32**********/	
    void handleRequest0x000102F3(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateAllZeroPointOffsets, RequestMessageType_Int32**********/	
    void handleRequest0x00011B03(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateSingleZeroPointOffset, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x000131D4(int recv_bytes);
    /********rpc/motion_control/axis_group/calibrateZeroPointOffsets, RequestMessageType_Int32_Int32List(count=9)**********/	
    void handleRequest0x00005AE3(int recv_bytes);   
    /********rpc/motion_control/axis_group/setJointManualStep, RequestMessageType_Int32_DoubleList(DoubleList_count=9)**********/	
    void handleRequest0x00018470(int recv_bytes);
    /********rpc/motion_control/axis_group/getJointManualStep, RequestMessageType_Int32**********/	
    void handleRequest0x00006D10(int recv_bytes);
    /********rpc/motion_control/axis_group/setCartesianManualStep, RequestMessageType_Int32_Double**********/	
    void handleRequest0x0000A420(int recv_bytes);
    /********rpc/motion_control/axis_group/getCartesianManualStep, RequestMessageType_Int32**********/	
    void handleRequest0x0000EAC0(int recv_bytes);
    /********rpc/motion_control/axis_group/setOrientationManualStep, RequestMessageType_Int32_Double**********/	
    void handleRequest0x00002940(int recv_bytes);
    /********rpc/motion_control/axis_group/getOrientationManualStep, RequestMessageType_Int32**********/	
    void handleRequest0x00016D20(int recv_bytes);
    /********rpc/motion_control/axis_group/getFcpBasePose, RequestMessageType_Int32**********/	
    void handleRequest0x000016B5(int recv_bytes);
    /********rpc/motion_control/axis_group/getTcpCurrentPose, RequestMessageType_Int32List(count=2)**********/	
    void handleRequest0x00003B45(int recv_bytes);
    /********rpc/motion_control/getPostureByJoint, RequestMessageType_Int32_DoubleList(DoubleList_count=9)**********/	
    void handleRequest0x0000EC64(int recv_bytes);
    /********rpc/motion_control/axis_group/setOfflineTrajectoryFile, RequestMessageType_String**********/	
    void handleRequest0x00011275(int recv_bytes);
    /********rpc/motion_control/axis_group/PrepareOfflineTrajectory, RequestMessageType_Void**********/	
    void handleRequest0x000051E9(int recv_bytes);
    /********rpc/motion_control/axis_group/moveOfflineTrajectory, RequestMessageType_Void**********/	
    void handleRequest0x0000C4D9(int recv_bytes);

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

    /********rpc/interpreter/start, RequestMessageType_String**********/	
    void handleRequest0x00006154(int recv_bytes);
    /********rpc/interpreter/pause, RequestMessageType_Int32**********/	
    void handleRequest0x0000BA55(int recv_bytes);
    /********rpc/interpreter/resume, RequestMessageType_Int32**********/	
    void handleRequest0x0000CF55(int recv_bytes);
    /********rpc/interpreter/abort, RequestMessageType_Int32**********/	
    void handleRequest0x000086F4(int recv_bytes);
    /********rpc/interpreter/forward, RequestMessageType_Int32**********/	
    void handleRequest0x0000D974(int recv_bytes);
    /********rpc/interpreter/backward, RequestMessageType_Int32**********/	
    void handleRequest0x00008E74(int recv_bytes);
    /********rpc/interpreter/jump, RequestMessageType_Int32**********/	
    void handleRequest0x00015930(int recv_bytes);


/* request end */

    /********rpc/publish/addTopic, ResponseMessageType_Uint64**********/	
    void handleResponse0x000050E3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/publish/deleteTopic, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004403(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
        
    /********rpc/tp_comm/getRpcTable, ResponseMessageType_Uint64_RpcTable**********/	
    void handleResponse0x00004FA5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tp_comm/getPublishTable, ResponseMessageType_Uint64_PublishTable**********/	
    void handleResponse0x000147A5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
        
    /********rpc/file_manager/readFile, ResponseMessageType_Uint64_Bytes**********/	
    void handleResponse0x0000A545(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/file_manager/writeFile, ResponseMessageType_Uint64**********/	
    void handleResponse0x00010D95(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
        
    /********rpc/controller/getVersion, ResponseMessageType_Uint64_Uint32List(count=4)**********/	
    void handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/setSystemTime, ResponseMessageType_Uint64**********/	
    void handleResponse0x000167C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getSystemTime, ResponseMessageType_Uint64_Uint64**********/	
    void handleResponse0x000003F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/setWorkMode, ResponseMessageType_Uint64**********/	
    void handleResponse0x00006825(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getWorkMode, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x00003325(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/setControlMode, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000B555(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/controller/getControlMode, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x0000B695(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/axis/mcPower, ResponseMessageType_Uint64**********/	
    void handleResponse0x000053E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReset, ResponseMessageType_Uint64**********/	
    void handleResponse0x000180C4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcStop, ResponseMessageType_Uint64**********/	
    void handleResponse0x00002820(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcHalt, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004BB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcSetPosition, ResponseMessageType_Uint64**********/	
    void handleResponse0x0001798E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadParameter, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x00016BF2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcWriteParameter, ResponseMessageType_Uint64**********/	
    void handleResponse0x00005732(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcMoveAbsolute, ResponseMessageType_Uint64**********/	
    void handleResponse0x000051F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcMoveVelocity, ResponseMessageType_Uint64**********/	
    void handleResponse0x00016CF9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadActualPosition, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x000012BE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadActualVelocity, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x00002EA9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadActualTorque, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x00014265(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadAxisInfo, ResponseMessageType_Uint64_AxisInfo**********/	
    void handleResponse0x0000314F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadStatus, ResponseMessageType_Uint64_AxisStatus**********/	
    void handleResponse0x00003E53(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadAxisError, ResponseMessageType_Uint64_Uint64**********/	
    void handleResponse0x000063C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcReadAxisErrorHistory, ResponseMessageType_Uint64_Uint64List(count=8)**********/	
    void handleResponse0x00018469(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcMoveRelative, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000CC85(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/mcHome, ResponseMessageType_Uint64**********/	
    void handleResponse0x000059B5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/rtmAbortHoming, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000E4B7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/rtmReadAxisFdbPdoPtr, ResponseMessageType_Uint64_Int32List**********/	
    void handleResponse0x0000A632(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/axis/rtmResetEncoder, ResponseMessageType_Uint64**********/	
    void handleResponse0x00000BA2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/group/mcGroupReset, ResponseMessageType_Uint64**********/	
    void handleResponse0x00016FF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/group/mcGroupEnable, ResponseMessageType_Uint64**********/	
    void handleResponse0x00003615(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/group/mcGroupDisable, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000D185(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/group/mcGroupReadError, ResponseMessageType_Uint64_Uint64**********/	
    void handleResponse0x00004BE2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/group/mcGroupReadStatus, ResponseMessageType_Uint64_GroupStatus**********/	
    void handleResponse0x00002A83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/group/resetAllEncoder, ResponseMessageType_Uint64**********/	
    void handleResponse0x000019D2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/servo_sampling/setSamplingConfiguration, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000845E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/getSamplingConfiguration, ResponseMessageType_Uint64_Uint32List(count=2)**********/	
    void handleResponse0x000106EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/activateSamplingConfiguration, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000CDDE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/setSamplingSync, ResponseMessageType_Uint64**********/	
    void handleResponse0x00003743(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/getSamplingSync, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x00006343(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/setSamplingChannel, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000BACC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/getSamplingChannel, ResponseMessageType_Uint64_Uint32List(count=16)**********/	
    void handleResponse0x0000556C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo_sampling/saveSamplingBufferData, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004E41(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
        
    /********rpc/servo1001/servo/shutDown, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000863E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/switchOn, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000E5CE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/disableVoltage, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004755(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/enableOperation, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000313E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/switchOnAndEnableOperation, ResponseMessageType_Uint64**********/	
    void handleResponse0x000177CE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/disableOperation, ResponseMessageType_Uint64**********/	
    void handleResponse0x000026AE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/quickStop, ResponseMessageType_Uint64**********/	
    void handleResponse0x00000580(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/resetFault, ResponseMessageType_Uint64**********/	
    void handleResponse0x00010584(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/transCommState, ResponseMessageType_Uint64**********/	
    void handleResponse0x000153C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/readParameter, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x00006892(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/writeParameter, ResponseMessageType_Uint64**********/	
    void handleResponse0x00007C32(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/moveVelocity, ResponseMessageType_Uint64**********/	
    void handleResponse0x000164D9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/moveAbsolute, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004DD5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/triggerUploadParameters, ResponseMessageType_Uint64**********/	
    void handleResponse0x000020B3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/uploadParameters, ResponseMessageType_Uint64_Int32List**********/	
    void handleResponse0x0000E003(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/triggerDownloadParameters, ResponseMessageType_Uint64**********/	
    void handleResponse0x00011C53(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/downloadParameters, ResponseMessageType_Uint64**********/	
    void handleResponse0x00017063(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/isAsyncServiceFinish, ResponseMessageType_Uint64_Bool**********/	
    void handleResponse0x000043B8(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/getCommState, ResponseMessageType_Uint64_CoreCommState**********/	
    void handleResponse0x0000F485(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/getServoState, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x000032F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/servo/moveRelative, ResponseMessageType_Uint64**********/	
    void handleResponse0x000172C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/resetEncoder, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000EFE2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/goHome, ResponseMessageType_Uint64**********/	
    void handleResponse0x00013BB5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/abortHoming, ResponseMessageType_Uint64**********/	
    void handleResponse0x00015AB7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/servo1001/cpu/getVersion, ResponseMessageType_Uint64_Uint32List(count=2)**********/	
    void handleResponse0x0001192E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/setCtrlPdoSync, ResponseMessageType_Uint64**********/	
    void handleResponse0x00005123(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getCtrlPdoSync, ResponseMessageType_Uint64_Int32_Uint32**********/	
    void handleResponse0x00005463(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/setSamplingSync, ResponseMessageType_Uint64**********/	
    void handleResponse0x00004023(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getSamplingSync, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x00006C23(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/setSamplingInterval, ResponseMessageType_Uint64**********/	
    void handleResponse0x00003EEC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getSamplingInterval, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x00001C2C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/setSamplingMaxTimes, ResponseMessageType_Uint64**********/	
    void handleResponse0x000110A3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getSamplingMaxTimes, ResponseMessageType_Uint64_Uint32**********/	
    void handleResponse0x00013363(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/setSamplingChannel, ResponseMessageType_Uint64**********/	
    void handleResponse0x00008E5C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getSamplingChannel, ResponseMessageType_Uint64_Uint32List(count=16)**********/	
    void handleResponse0x0000FD9C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/activateSamplingConfiguration, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000939E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/saveSamplingBufferData, ResponseMessageType_Uint64**********/	
    void handleResponse0x00015621(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/servo/getServoCommInfo, ResponseMessageType_Uint64_Int32List(count=6)**********/	
    void handleResponse0x0000BF1F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/servo1001/cpu/getServoCpuCommInfo, ResponseMessageType_Uint64_Int32List(count=2)**********/	
    void handleResponse0x0000FE5F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);	
    /********rpc/servo1001/servo/getServoDefinedInfo, ResponseMessageType_Uint64_Int32List(count=9)**********/	
    void handleResponse0x0000C87F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********rpc/io/readDI, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x000185A9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io/readDO, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x000185AF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/io/writeDO, ResponseMessageType_Uint64**********/	
    void handleResponse0x00000C1F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);


    /********rpc/tool_manager/addTool, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000A22C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tool_manager/deleteTool, ResponseMessageType_Uint64**********/	
    void handleResponse0x00010E4C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tool_manager/updateTool, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000C78C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tool_manager/moveTool, ResponseMessageType_Uint64**********/	
    void handleResponse0x000085FC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tool_manager/getToolInfoById, ResponseMessageType_Uint64_ToolInfo**********/	
    void handleResponse0x00009E34(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/tool_manager/getAllValidToolSummaryInfo, ResponseMessageType_Uint64_ToolSummaryList**********/	
    void handleResponse0x0001104F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/coordinate_manager/addUserCoord, ResponseMessageType_Uint64**********/	
    void handleResponse0x00016764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/coordinate_manager/deleteUserCoord, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000BAF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/coordinate_manager/updateUserCoord, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000EC14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/coordinate_manager/moveUserCoord, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000E104(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/coordinate_manager/getUserCoordInfoById, ResponseMessageType_Uint64_UserCoordInfo**********/	
    void handleResponse0x00004324(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/coordinate_manager/getAllValidUserCoordSummaryInfo, ResponseMessageType_Uint64_UserCoordSummaryList**********/	
    void handleResponse0x0001838F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/motion_control/setGlobalVelRatio, ResponseMessageType_Uint64**********/	
    void handleResponse0x000005EF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getGlobalVelRatio, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x0001578F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/setGlobalAccRatio, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000271F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getGlobalAccRatio, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x00016D9F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doStepManualMove, ResponseMessageType_Uint64**********/	
    void handleResponse0x000085D5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doContinuousManualMove, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000D3F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doGotoCartesianPointManualMove, ResponseMessageType_Uint64**********/	
    void handleResponse0x00010C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doGotoJointPointManualMove, ResponseMessageType_Uint64**********/	
    void handleResponse0x00008075(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/doManualStop, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000A9A0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getJointsFeedBack, ResponseMessageType_Uint64_DoubleList(count = 9) **********/	
    void handleResponse0x0000DFBB(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setUserSoftLimit, ResponseMessageType_Uint64**********/	
    void handleResponse0x000114A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getUserSoftLimit, ResponseMessageType_Uint64_JointLimit**********/	
    void handleResponse0x0000C764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setManuSoftLimit, ResponseMessageType_Uint64**********/	
    void handleResponse0x000108E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getManuSoftLimit, ResponseMessageType_Uint64_JointLimit **********/	
    void handleResponse0x0000C244(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setHardLimit, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000C454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getHardLimit, ResponseMessageType_Uint64_JointLimit **********/	
    void handleResponse0x00013394(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setCoordinate, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000A845(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getCoordinate, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x00008595(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setUserCoordId, ResponseMessageType_Uint64**********/	
    void handleResponse0x00005CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getUserCoordId, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x00005BB4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setTool, ResponseMessageType_Uint64**********/	
    void handleResponse0x0001581C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getTool, ResponseMessageType_Uint64_Int32**********/	
    void handleResponse0x0001354C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/convertCartToJoint, ResponseMessageType_Uint64_DoubleList(count = 9)**********/	
    void handleResponse0x00010FD4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/convertJointToCart, ResponseMessageType_Uint64_PoseAndPosture**********/	
    void handleResponse0x0000B6D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/ignoreLostZeroError, ResponseMessageType_Uint64**********/	
    void handleResponse0x00014952(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setSingleZeroPointOffset, ResponseMessageType_Uint64**********/	
    void handleResponse0x00012404(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setAllZeroPointOffsets, ResponseMessageType_Uint64**********/	
    void handleResponse0x00011853(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getAllZeroPointOffsets, ResponseMessageType_Uint64_DoubleList(count=9)**********/	
    void handleResponse0x00012353(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getAllZeroErrorMaskStatus, ResponseMessageType_Uint64_Int32List(count=9)**********/	
    void handleResponse0x0000C183(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setSingleZeroPointStatus, ResponseMessageType_Uint64**********/	
    void handleResponse0x00010E43(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getAllZeroPointStatus, ResponseMessageType_Uint64_Int32List(count=9)**********/	
    void handleResponse0x000102F3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateAllZeroPointOffsets, ResponseMessageType_Uint64**********/	
    void handleResponse0x00011B03(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateSingleZeroPointOffset, ResponseMessageType_Uint64**********/	
    void handleResponse0x000131D4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/calibrateZeroPointOffsets, ResponseMessageType_Uint64**********/	
    void handleResponse0x00005AE3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setJointManualStep, ResponseMessageType_Uint64**********/	
    void handleResponse0x00018470(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getJointManualStep, ResponseMessageType_Uint64_DoubleList(DoubleList_count=9)**********/	
    void handleResponse0x00006D10(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setCartesianManualStep, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000A420(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getCartesianManualStep, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x0000EAC0(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setOrientationManualStep, ResponseMessageType_Uint64**********/	
    void handleResponse0x00002940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getOrientationManualStep, ResponseMessageType_Uint64_Double**********/	
    void handleResponse0x00016D20(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getFcpBasePose, ResponseMessageType_Uint64_DoubleList(count=6)**********/	
    void handleResponse0x000016B5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/getTcpCurrentPose, ResponseMessageType_Uint64_DoubleList(count=6)**********/	
    void handleResponse0x00003B45(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/getPostureByJoint, ResponseMessageType_Uint64_Posture**********/	
    void handleResponse0x0000EC64(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/setOfflineTrajectoryFile, ResponseMessageType_Uint64**********/	
    void handleResponse0x00011275(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/PrepareOfflineTrajectory, ResponseMessageType_Uint64**********/	
    void handleResponse0x000051E9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/motion_control/axis_group/moveOfflineTrajectory, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000C4D9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/reg_manager/pr/addReg, ResponseMessageType_Uint64**********/	
    void handleResponse0x000154E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/deleteReg, ResponseMessageType_Uint64**********/	
    void handleResponse0x00001097(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/updateReg, ResponseMessageType_Uint64**********/	
    void handleResponse0x00009EF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getReg, ResponseMessageType_Uint64_PrRegData**********/	
    void handleResponse0x00017207(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/moveReg, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000D7C7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getChangedList, ResponseMessageType_Uint64_BaseRegSummaryList**********/	
    void handleResponse0x0000B454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/reg_manager/pr/getValidList, ResponseMessageType_Uint64_BaseRegSummaryList**********/	
    void handleResponse0x00009354(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********rpc/interpreter/start, ResponseMessageType_Uint64**********/	
    void handleResponse0x00006154(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/pause, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000BA55(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/resume, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000CF55(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/abort, ResponseMessageType_Uint64**********/	
    void handleResponse0x000086F4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/forward, ResponseMessageType_Uint64**********/	
    void handleResponse0x0000D974(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/backward, ResponseMessageType_Uint64**********/	
    void handleResponse0x00008E74(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);
    /********rpc/interpreter/jump, ResponseMessageType_Uint64**********/	
    void handleResponse0x00015930(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size);

    /********publish/axis/feedback, MessageType_AxisFeedbackList(count=14)**********/    
    void handlePublishElement0x0001715B(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********publish/servo1001/servo/feedback, MessageType_Servo100servoFeedbackList(count=14)**********/    
    void handlePublishElement0x0001128B(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********publish/servo1001/cpu/feedback, MessageType_Uint32List(count=15)**********/ 
    void handlePublishElement0x00012FFB(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********publish/io1000/io_feedback, MessageType_Uint32List(count=4)**********/	
    void handlePublishElement0x00013C8B(Comm_Publish& package, int element_index, TpPublishElement& list_element);
    /********publish/ioAnalog/io_analog_feedback, MessageType_Uint32List(count=12)**********/	
    void handlePublishElement0x00007C5B(Comm_Publish& package, int element_index, TpPublishElement& list_element);


    /* response end */
    base_space::LocalIP local_ip_;
    TpCommManagerConfig* param_ptr_;

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

	//event
	int send_event_socket_;
	int send_event_endpoint_id_;
	std::string send_event_ip_;
	struct nn_pollfd poll_send_event_fd_;

    base_space::ThreadHelp thread_ptr_;

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
	std::mutex send_event_list_mutex_;
    std::list<TpRequestResponse>  request_list_;
    std::vector<TpRequestResponse>  response_list_;
    std::vector<TpPublish>  publish_list_;
	std::vector<TpEventMsg> send_event_list_;
};
}
#endif

void* tpCommRoutineThreadFunc(void* arg);
