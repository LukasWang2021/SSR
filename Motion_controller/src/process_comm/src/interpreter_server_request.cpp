#include "interpreter_server.h"
#include "interpreter_common.h"

using namespace fst_base;


// Start
void InterpreterServer::handleRequestStart()
{
    char* request_data_ptr = new char[256]();
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, 256);
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_START, (void*)request_data_ptr, (void*)response_data_ptr);        
}

// Launch
void InterpreterServer::handleRequestLaunch()
{
    char* request_data_ptr = new char[256]();
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete[] request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, 256);
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_LAUNCH, (void*)request_data_ptr, (void*)response_data_ptr);   
}

// Forward
void InterpreterServer::handleRequestForward()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_FORWARD, NULL, (void*)response_data_ptr);  
}

// Backward
void InterpreterServer::handleRequestBackward()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_BACKWARD, NULL, (void*)response_data_ptr);  
}

// Jump
void InterpreterServer::handleRequestJump()
{
    char* request_data_ptr = new char[512]();
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete[] request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, 512);
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_JUMP, (void*)request_data_ptr, (void*)response_data_ptr);  
}

// Pause
void InterpreterServer::handleRequestPause()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_PAUSE, NULL, (void*)response_data_ptr); 
}

// Continue
void InterpreterServer::handleRequestResume()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_RESUME, NULL, (void*)response_data_ptr); 
}

// Abort
void InterpreterServer::handleRequestAbort()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_ABORT, NULL, (void*)response_data_ptr); 
}

// GetNextInstruction
void InterpreterServer::handleRequestGetNextInstruction()
{
    Instruction* response_data_ptr = new Instruction;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION, NULL, (void*)response_data_ptr); 
}

// CodeStart
void InterpreterServer::handleRequestCodeStart()
{
    char* request_data_ptr = new char[256]();
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete[] request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, 256);
    pushTaskToRequestList(INTERPRETER_SERVER_CMD_CODE_START, (void*)request_data_ptr, (void*)response_data_ptr); 
}