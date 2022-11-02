#include "init_protector.h"
#include <iostream>
#include <cstdio>
#include <string>
#include <fstream>

using namespace std;

#define PROCESS_ADDRESS "/tmp/"
#define FILE_TYPE ".initpro"

bool user_space::init_protect(const char* f_name)
{
    string rec_name = f_name;
    rec_name += FILE_TYPE;
    string file_location = PROCESS_ADDRESS + rec_name;

    // check whether the file exist, if not, create one
    if(access(file_location.c_str(), F_OK) != 0)
    {
        ofstream lout(file_location.c_str());
        cout << "INIT_PROTECTOR -> SUCCESS: " << f_name << endl;
    }
    else
    {
        cout << "INIT_PROTECTOR -> ERROR: " << rec_name << " founded" << endl;
        cout << "Please clean /tmp or check the running process" << endl;
        return false;
    }

    return true;
}

void user_space::init_clean()
{
    string s = "rm -f ";
    s += PROCESS_ADDRESS;
    string temp = "*.initpro";
    s += temp;
    system(s.c_str());
}
// handle signal SIGINT (ctrl+c)
void user_space::init_signalHandler(int signum)
{
        cout<<endl<<"INIT_PROTECTOR -> INFO: program ended by ctrl+c (SIGINT)"<<endl;
        init_clean();
        exit(signum);
}

// handle signal SIGTERM (command kill)
void user_space::init_signalHandler2(int signum)
{
    cout<<endl<<"INIT_PROTECTOR -> INFO: program ended by kill command (SIGTERM)"<<endl;
    user_space::init_clean();
    std::exit(signum);
}

// handle signal SIGHUP (close terminal window)
void user_space::init_signalHandler3(int signum)
{
    cout<<endl<<"INIT_PROTECTOR -> INFO: program ended by close terminal windows (SIGHUP)"<<endl;
    user_space::init_clean();
    std::exit(signum);
}