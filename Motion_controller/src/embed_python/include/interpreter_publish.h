#ifndef INTERPRETER_PUBLISH_H
#define INTERPRETER_PUBLISH_H

#include "interpreter_control.h"


typedef struct
{
    int curr_line;
    std::string curr_prog;
    InterpState curr_state;
    InterpMode curr_mode;
}InterpPubData;

class InterpPublish
{
private:
    InterpPubData pub_data_;
public:
    InterpPublish(/* args */);
    ~InterpPublish();

    InterpPubData getPubData(void);
};



#endif
