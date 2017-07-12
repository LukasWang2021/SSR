/**
 * @file error_process.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */
#include "error_process.h"

ErrorProcess::ErrorProcess()
{

}
ErrorProcess::~ErrorProcess()
{

}

U64 ErrorProcess::getInfoCode()
{
    return err_info_;
}

U64 ErrorProcess::getPauseCode()
{
    return err_pause_;
}

U64 ErrorProcess::getStopCode()
{
    return err_stop_;
}

U64 ErrorProcess::getServo01Code()
{
    return err_servo01_;
}

U64 ErrorProcess::getServo02Code()
{
    return err_servo02_;
}

U64 ErrorProcess::getAbortCode()
{
    return err_abort_;
}

U64 ErrorProcess::getSystemCode()
{
    return err_system_;
}


bool ErrorProcess::setInfoCode(U64 code)
{
    if (err_info_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setPauseCode(U64 code)
{
    if (err_pause_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setStopCode(U64 code)
{
    if (err_stop_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setServo01Code(U64 code)
{
    if (err_servo01_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setServo02Code(U64 code)
{
    if (err_servo02_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setAbortCode(U64 code)
{
    if (err_abort_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}

bool ErrorProcess::setSystemCode(U64 code)
{
    if (err_system_ == code)
    {
        return false;
    }
    else
    {
        err_info_ = code;
        return true;
    }
}


