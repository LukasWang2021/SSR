/*************************************************************************
	> File Name: manager_node.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 13时53分03秒
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <ctime>

#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_param_builder.h>

#include <ros/ros.h>
#include <ros/param.h>
#include <boost/filesystem.hpp>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;



int main(int argc, char **argv)
{
	cout << "hello world" << endl;

	int loop = 0;
    int i;
    while (i < 100) {
	while (loop < 10) {
		loop++;
		fst_parameter::ParamValue pv;
		fst_parameter::ParamBuilder builder;
		char buf[128];
		snprintf(buf, sizeof(buf), "/root/install/share/configuration/test%d.yaml", loop);
        cout << buf << endl;
		std::ifstream in(buf);
        cout << "  open";
		string yaml_str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        cout << "  read";
		//cout << "string :" << endl << yaml_str << endl;
		//cout << "Build param_value from YAML" << endl;
        try {
		    builder.buildParamFromString(yaml_str, pv);
		}
        catch (fst_parameter::ParamException &e) {
            cout << "parse failed." << endl;
			cout << e.getMessage() << endl;
			// cout << pv << endl;
		}

		string dump;
        try {
		    builder.dumpParamToString(pv, dump);
            // cout << dump << endl;
		}
        catch (fst_parameter::ParamException &e) {
            cout << "dump failed." << endl;
			cout << e.getMessage() << endl;
			// cout << pv << endl;
		}

		getchar();
	}
    }

	return 0;
}




