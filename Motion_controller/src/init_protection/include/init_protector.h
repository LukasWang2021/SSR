#pragma once
#include <iostream>
#include <string>
#include <csignal>
#include <unistd.h>

using namespace std;


namespace user_space
{
    bool init_protect(const char* f_name);
    void init_clean();
    void init_signalHandler(int signum);
    void init_signalHandler2(int signum);
    void init_signalHandler3(int signum);
}