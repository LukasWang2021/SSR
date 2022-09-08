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
        cout<<"INIT_PROTECTOR -> SUCCESS: "<<rec_name<<" does not exist"<<endl;
        cout<<"INIT_PROTECTOR -> SUCCESS: new "<<rec_name<<" has now been created in tmp"<<endl;
    }
    else
    {
        cout<<"INIT_PROTECTOR -> WARNING: "<<rec_name<<" founded"<<endl;
        return false;
    }

    return true;
}

void user_space::init_clean()
{
    string s = "rm ";
    s += PROCESS_ADDRESS;
    string temp = "*.initpro";
    s += temp;
    system(s.c_str());
}

void user_space::init_signalHandler(int signum)
{
        cout<<endl<<"INIT_PROTECTOR -> INFO: program ended by ctrl+c (SIGINT)"<<endl;
        init_clean();
        exit(signum);
}

void user_space::init_signalHandler2(int signum)
{
    cout<<endl<<"INIT_PROTECTOR -> INFO: program ended by kill command (SIGTERM)"<<endl;
    user_space::init_clean();
    std::exit(signum);
}