#include "anybus_manager.h"
#include <iostream>

using namespace std;
using namespace fst_anybus;

#define ABIP_LINUX_USE_MII_ETH_IF   1
#define ABIP_LINUX_MII_ETH_IF       "eth1"

int main( int argc, char **argv )
{
    AnybusManager anybus_manager;
    if( 0 != anybus_manager.init())
    {
        cout << "failed to init anybus_manager" << endl;
        return -1;
    }

    if( 0 != anybus_manager.initAnybus())
    {
        cout << "failed to init anybus library" << endl;
        return -1;
    }

    if( 0 != anybus_manager.openServer())
    {
        cout << "failed to open anybus server" << endl;
        return -1;
    }

    while(true)
    {
        sleep(2);
    }

    return 0;
}

