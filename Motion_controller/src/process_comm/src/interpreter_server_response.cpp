#include "interpreter_server.h"
#include <unistd.h>

using namespace fst_base;


// Start
void InterpreterServer::handleResponseStart(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_START, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Debug
void InterpreterServer::handleResponseDebug(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_DEBUG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Forward
void InterpreterServer::handleResponseForward(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_FORWARD, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Backward
void InterpreterServer::handleResponseBackward(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_BACKWARD, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Jump
void InterpreterServer::handleResponseJump(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_JUMP, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Pause
void InterpreterServer::handleResponsePause(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_PAUSE, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Continue
void InterpreterServer::handleResponseResume(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_RESUME, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// Abort
void InterpreterServer::handleResponseAbort(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_ABORT, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// GetNextInstruction
void InterpreterServer::handleResponseGetNextInstruction(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetAutoStartMode
void InterpreterServer::handleResponseSetAutoStartMode(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

