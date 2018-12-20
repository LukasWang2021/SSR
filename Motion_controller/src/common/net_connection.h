#ifndef NET_CONNECTION_HPP
#define NET_CONNECTION_HPP

#include <iostream>

using namespace std;

class NetConnection
{
public:
    NetConnection()
    {
        status_ = false;
    }

    ~NetConnection(){}

    bool set(bool status)
    { 
        status_ = status;
        return true;
    }
    bool get() { return status_; }

private:
    bool status_;
};
#endif


