#include "interpreter_publish.h"
#include "interpreter_control.h"


InterpPublish::InterpPublish(/* args */)
{

}

InterpPublish::~InterpPublish()
{

}

InterpPubData InterpPublish::getPubData(void)
{
    InterpPubData data;
    data.curr_line = InterpCtrl::instance().getLineNumber();
    data.curr_prog = InterpCtrl::instance().getProgName();
    data.curr_mode = InterpCtrl::instance().getMode();
    data.curr_state = InterpCtrl::instance().getState();
    return data;
}