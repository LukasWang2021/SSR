/*************************************************************************
	> File Name: manager_node.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 13时53分03秒
 ************************************************************************/
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<math.h>
#include<time.h>
#include <iostream>
#include <fstream>
#include <ctime>

#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_param_builder.h>

#include <ros/ros.h>
#include <ros/param.h>
#include <boost/filesystem.hpp>

#define PI  3.1415926

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;

void test1(void)
{
    clock_t start, finish;

    printf(">>>>>>>>>>>test1<<<<<<<<<<<<<\n");

    double matrix[4][4];
    double t;

    start = clock();
    for (int i = 0; i < 1000000; i++) {
        t = PI * i / 1000000;

        double tmp[4][4] = {    {cos(t),    0,  sin(t),  0},
                                {0,         1,  0,       0},
                                {-sin(t),   0,  cos(t),  0},
                                {0,         0,  0,       1}
                            };
        
        for (int m = 0; m < 4; m++)
            for (int n = 0; n < 4; n++)
                matrix[m][n] = tmp[m][n];
    }
    finish = clock();
    printf("time consumed by origin alg: %fs\n", (double)(finish - start) / CLOCKS_PER_SEC);

    start = clock();
    for (int i = 0; i < 1000000; i++) {
        t = PI * i / 1000000;

        double a = cos(t);
        double b = sin(t);

        matrix[0][0] = a;
        matrix[0][1] = 0;
        matrix[0][2] = b;
        matrix[0][3] = 0;
        matrix[1][0] = 0;
        matrix[1][1] = 1;
        matrix[1][2] = 0;
        matrix[1][3] = 0;
        matrix[2][0] = -b;
        matrix[2][1] = 0;
        matrix[2][2] = a;
        matrix[2][3] = 0;
        matrix[3][0] = 0;
        matrix[3][1] = 0;
        matrix[3][2] = 0;
        matrix[3][3] = 1;
    }
    finish = clock();
    printf("time consumed by new alg 1: %fs\n", (double)(finish - start) / CLOCKS_PER_SEC);

    double *mat = matrix[0];
    start = clock();
    for (int i = 0; i < 1000000; i++) {
        t = PI * i / 1000000;

        double a = cos(t);
        double b = sin(t);
        
        mat[0] = a;
        mat[1] = 0;
        mat[2] = b;
        mat[3] = 0;
        mat[4] = 0;
        mat[5] = 1;
        mat[6] = 0;
        mat[7] = 0;
        mat[8] = -b;
        mat[9] = 0;
        mat[10] = a;
        mat[11] = 0;
        mat[12] = 0;
        mat[13] = 0;
        mat[14] = 0;
        mat[15] = 1;
    }
    finish = clock();
    printf("time consumed by new alg 2: %fs\n", (double)(finish - start) / CLOCKS_PER_SEC);

    double *p = new double[16];
    memcpy(p, mat, 16 *sizeof(double));
    delete [] p;
}

int main(int argc, char **argv)
{
	cout << "hello world" << endl;

    test1();

    while(1);

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




