/*************************************************************************
	> File Name: manager_node.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 13时53分03秒
 ************************************************************************/

#include <iostream>
#include <fstream>
#include <ctime>
// #include <yaml-cpp/yaml.h>

// #include <parameter_manager/parameter_manager_param_type.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_param_value.h>
#include <parameter_manager/parameter_manager_param_builder.h>


#include <ros/ros.h>
#include <ros/param.h>
#include <boost/filesystem.hpp>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;


void test_3(const string &file);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    string file_name = "share/motion_controller/config/motion_controller.yaml";
    //fst_parameter::ParamGroup params("/home/fst/ros_workspace/src/parameter_manager/config/test.yaml");
    //fst_parameter::ParamGroup param(file_name);
    
    string yaml_str;
        char temp[1024] = {0};
        int length = readlink("/proc/self/exe", temp, sizeof(temp));
        if (length > 0 && length < sizeof(temp)) {
            boost::filesystem::path executable(temp);
            file_name = executable.parent_path().parent_path().parent_path().string() + "/" + file_name;
        }
        std::ifstream yaml_handle_r(file_name.c_str());
        if (yaml_handle_r.is_open()) {
            string temp_str((std::istreambuf_iterator<char>(yaml_handle_r)),
                             std::istreambuf_iterator<char>());
            yaml_str = temp_str;
            yaml_handle_r.close();
        }
    cout << "yaml_str:\n" << yaml_str << endl;
    fst_parameter::ParamValue pv;
    fst_parameter::ParamBuilder builder;
    builder.buildParamFromString(yaml_str, pv);

    //builder.buildParamFromString("{arm_group:{enable_calibration:true,enable_logger:false,hello,trajectory_fifo_length:100},calibrator:{lost_offset_threshold:[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1],normal_offset_threshold:[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]},planning_interface:{acceleration:6000,acceleration_scaling:1,alpha_overload:3,cycle_time:0.001,jerk:10,joint_errorangle:0.15,joint_overshoot:0.25,omega_overload:1,smooth_curve:{cubic:2,quadratic:1},smooth_curve_mode:quadratic,smooth_radius_coefficient:1.2,velocity:1000,velocity_scaling:1}}", pv);
    //builder.buildParamFromString("  \r \r\n\n\rname : feng\nage:19\nsex   :           male\n\r\rfamily:\n  children: Tom\n  wife: huang\n  ", pv);
    //builder.buildParamFromString("test:{age:19,family:{children:[[Tom, Jim],[Sunny, Jessy],[Sue]],wife:huang},name:feng,sex:male}", pv);
    //builder.buildParamFromString("test:{age:19,family:{children:[Tom, Jim,Sunny, Jessy],wife:huang},name:feng,sex:male}", pv);

    std::cout << pv << std::endl;

    /*
    time_t begin, end;
    vector<int> vec(1000000, 5);
    vector<int>::iterator it = vec.begin();

    begin = clock();
    for(it = vec.begin(); it != vec.end(); it++) {
        *it = *it + 1;
    }
    end = clock();
    cout << "it++ runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;

    begin = clock();
    for(it = vec.begin(); it != vec.end(); ++it) {
        *it = *it + 1;
    }
    end = clock();
    cout << "++it runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;

    begin = clock();
    for(it = vec.begin(); it != vec.end(); it = it + 1) {
        *it = *it + 1;
    }
    end = clock();
    cout << "it+1 runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    */

    //XmlRpc::XmlRpcValue v;
    //ros::param::get("/fst_param", v);
    //cout << v << endl;
    //cout << v.toXml() << endl;

    // test_1(params);
    // test_2(params);
    // test_3(file_name);

}

void test_3(const string &file) {
    time_t begin, end;
    ROS_INFO("begin to test loadParamFile");
    for (int loop = 0; loop < 100; ++loop) {
        begin = clock();
        fst_parameter::ParamGroup param(file);
        end = clock();
        cout << "runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    }
    ROS_INFO("end");
}

/*
void test_1(fst_parameter::ParamGroup &params) {
    time_t begin, end;
    int test;
    ROS_INFO("get int value using ParamValue");
    for (int loop = 0; loop < 100; ++loop) {
        begin = clock();
        for (int cnt = 0; cnt < 10000; ++cnt) {
            params.getParam("house/address/number", test);
        }
        end = clock();
        cout << "runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    }
    ROS_INFO("getted %d", test);
    
    ROS_INFO("get int value using YAML::Node");
    for (int loop = 0; loop < 100; ++loop) {
        begin = clock();
        for (int cnt = 0; cnt < 10000; ++cnt) {
            params.getParam_1("house/address/number", test);
        }
        end = clock();
        cout << "runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    }
    ROS_INFO("getted %d", test);
}

void test_2(fst_parameter::ParamGroup &params) {
    time_t begin, end;
    int test = 50;
    ROS_INFO("set int value using ParamValue");
    for (int loop = 0; loop < 100; ++loop) {
        begin = clock();
        for (int cnt = 0; cnt < 10000; ++cnt) {
            params.setParam("house/address/number", test);
        }
        end = clock();
        cout << "runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    }
    test = 0;
    params.getParam("house/address/number", test);
    ROS_INFO("setted %d", test);
    
    ROS_INFO("set int value using YAML::Node");
    for (int loop = 0; loop < 100; ++loop) {
        begin = clock();
        for (int cnt = 0; cnt < 10000; ++cnt) {
            params.setParam_1("house/address/number", test);
        }
        end = clock();
        cout << "runtime:" << double(end - begin) / CLOCKS_PER_SEC << endl;
    }
    test = 0;
    params.getParam_1("house/address/number", test);
    ROS_INFO("setted %d", test);
}*/

/*
int main(int argc, char **argv)
{
    ros::init(argc, argv, "parameter_manager");
    string file_name = "config/motion_controller.yaml";

    cout<<"------------------- Load Param File Test -------------------"<<endl;
    // fst_parameter::ParamGroup param(file_name);
    fst_parameter::ParamGroup param_group("config/motion_controller.yaml");
    // fst_parameter::ParamGroup param_group;
    // param_group.loadParamFile(file_name);

    cout<<"------------------- Upload Param Test ----------------------"<<endl;
    cout << "upload ... " << (param_group.uploadParam()?"True":"False") << endl;
    
    cout<<"------------------- Get Remote Param Test -------------------"<<endl;
    int rpa = 0;
    cout << "*****" << param_group.getRemoteParam("name.Tome.age", rpa) <<  ", rpa=";
    cout << rpa << endl;
    vector<int> rpb;
    cout << "*****" << param_group.getRemoteParam("name/first/sda", rpb) <<  ", rpb=[";
    for (vector<int>::iterator it = rpb.begin(); it != rpb.end(); ++it) {
        cout << *it;
        if (it + 1 != rpb.end())
            cout << ", ";
    }
    cout << "]" << endl;
    map<string, XmlRpc::XmlRpcValue> rpc;
    cout << "*****" << param_group.getRemoteParam("name/Tome", rpc) <<  ", rpc:" << endl;
    for (map<string, XmlRpc::XmlRpcValue>::iterator it = rpc.begin(); it != rpc.end(); ++it) {
        cout << "  " << it->first << ": " << it->second << endl;
    }


    cout<<"------------------- Set Map Test -------------------"<<endl;
    XmlRpc::XmlRpcValue v;
    map<string, XmlRpc::XmlRpcValue> t_map;
    v = "Tongji";
    cout << "XmlRpcValue: " << endl << v.toXml() << endl;
    t_map.insert(std::pair<string, XmlRpc::XmlRpcValue>("school", v));
    v = 20;
    t_map.insert(std::pair<string, XmlRpc::XmlRpcValue>("age", v));
    v = 1.75;
    t_map.insert(std::pair<string, XmlRpc::XmlRpcValue>("tall", v));
    v = true;
    t_map.insert(std::pair<string, XmlRpc::XmlRpcValue>("male", v));
    cout << "set a map: " << param_group.setParam("name.Tome", t_map) << endl;

    cout<<"------------------- Get Map Test -------------------"<<endl;
    map<string, XmlRpc::XmlRpcValue>::iterator iter;
    if (param_group.getParam("name.Tome", t_map)) {
        for (iter = t_map.begin(); iter != t_map.end(); ++iter) {
            cout << iter->first << ": " << iter->second << endl;
        }
    }
    else {
        cout << "Fail to get a map" << endl;

    }
    

    cout<<"------------------- Set/Get vector Test -------------------"<<endl;
    vector<double> vec1;
    vec1.push_back(3.3);
    vec1.push_back(1.5);
    vec1.push_back(2.01);
    param_group.setParam("name.first.sdc", vec1);
    vec1.clear();
    if (param_group.getParam("name.first.sdc", vec1)) {
        cout << "get new vector: ";
        for (vector<double>::iterator itr = vec1.begin(); itr != vec1.end(); ++itr)
            cout << *itr << " ";
        cout << endl;
    }
    else
        cout << "Cannot get vector " << endl;
    

    cout<<"------------------- Has Param Test -------------------"<<endl;
    cout << "has param:" << param_group.hasParam("name/first/sdd") << endl;


    cout<<"------------------- Set/Get Int Test -------------------"<<endl;
    cout << "set param: " << param_group.setParam("name/second", 45) << endl;
    int res = 0;
    param_group.getParam("name/second", res);
    cout << "name/second = " << res << endl;
    
    cout<<"------------------- Set/Get Double Test -------------------"<<endl;
    cout << "set param: " << param_group.setParam("name/first/sck", 4.5) << endl;
    double b;
    if (param_group.getParam("name/first/sck", b))
        cout << "get a param: " << b << endl;
    else
        cout << "Cannot get param: " << endl;

    // cout << param_group.setParam("name/first/sck", 123) << endl;
    // int res;
    // param_group.getParam("name/first/sck", res);
    // cout << "name/first/sck now is:" << res << endl;

    cout<<"------------------- Get Vector Test -------------------"<<endl;
    vector<int> vec;
    if (param_group.getParam("name/first/sda", vec)) {
        cout << "get a vector: ";
        for (vector<int>::iterator itr = vec.begin(); itr != vec.end(); ++itr)
            cout << *itr << " ";
        cout << endl;
    }
    else
        cout << "Cannot get vector " << endl;
    
    vector<double> vecb;
    if (param_group.getParam("name/first/sdc", vecb)) {
        cout << "get a vector: ";
        for (vector<double>::iterator itr = vecb.begin(); itr != vecb.end(); ++itr)
            cout << *itr << " ";
        cout << endl;
    }
    else
        cout << "Cannot get vector " << endl;
   
    cout<<"------------------- Some New Test -------------------"<<endl;
    cout << "hasParam:" << param_group.hasParam("name/first/sck") << endl;
    cout << "del:" << param_group.deleteParam("name/first/sck") << endl;
    cout << "hasParam:" << param_group.hasParam("name/first/sck") << endl;
    // param_group.test();
    
    cout<<"------------------- Dump File Test -------------------"<<endl;
    file_name = "config/motion_controller_dump.yaml";
    param_group.dumpParamFile(file_name);
*/
    /*
    cout<<"------------------- Test 1-------------------------"<<endl;
    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
        cout<<"Cannot open file"<<endl;
        return 0;
    }

    while (!file.eof()) {
        char buffer[200];
        file.getline(buffer, 200);
        cout<<buffer<<endl;
    }

    fst_parameter::Param param;
    param.type = fst_parameter::e_type_bool;
    param.value.as_bool = true;

    cout<< param << endl;
    */
    /*
    cout<<"------------------- Test 2-------------------------"<<endl;
    XmlRpc::XmlRpcValue v;
    std::string abb;
    ros::param::get("/test", v);
    cout << "type=" << v.getType();
    cout << "v=" << v << endl;
    int converted = int(v);
    cout << converted << endl;
    */
    /*
    YAML::Node con = YAML::LoadFile(file_name);
    cout << "con.type()=" << con.Type() << endl;
    YAML::const_iterator it = con.begin();
    cout << it->first << ":" << endl << it->second <<endl;
    cout << it->second.Type() << endl;

    YAML::Node n = it->second;
    cout << n.Type() << endl;
    YAML::Node a = n[0];
    cout << "a.type=" << a.Type() << endl << a << endl;
    YAML::const_iterator itr = a.begin();
    YAML::Node newbe = itr->second;
    cout << newbe.Scalar() << endl;
    cout << newbe.Tag() << endl;
    cout << newbe.Type() << endl;
    bool age = newbe.as<double>();
    cout << "age=" << age << endl;
    */
    /*
    YAML::Node config;
    try {
        config = YAML::LoadFile(file_name);
    }
    catch (YAML::BadFile& exception) {
        cout<< "bad file" << exception.what() << endl;
        return 0;
    }

    
    YAML::const_iterator it = config.begin();
    YAML::Node map = it->second;
    cout << "map type=" << map.Type() << endl;
    it = map.begin();
    cout << it->first << " is " << it->second << endl;
    cout << it->second.Type() << endl;
    double res = it->second.as<double>();
    cout << res << endl;
    std::string scalar = it->second.Scalar();
    cout << scalar << endl;
    */
/*
    cout<<"------------------- At Last -------------------------"<<endl;
    param_group.deleteParamTree();
    XmlRpc::XmlRpcValue value;
    value["one"] = 3.14;
    value["two"] = "student";
    value["three"] = true;
    cout << value << endl;
    XmlRpc::XmlRpcValue vmap;
    vmap["start"] = value;
    cout << vmap << endl;

    double one = value["one"];
    cout << one << endl;
    cout << sizeof(vmap) << endl;
    cout << sizeof(value) << endl;
   
    return 0;
}*/

/*
std::ostream& operator<<(std::ostream& os, fst_parameter::Param& v)
{
    switch (v.type) {
        case fst_parameter::e_type_bool:
            if (v.value.as_bool)
                os << "True";
            else
                os << "False";
            break;
        case fst_parameter::e_type_int:
            os << v.value.as_int;
            break;
        case fst_parameter::e_type_double:
            os << v.value.as_double;
            break;
        case fst_parameter::e_type_string:
            os << *v.value.as_string;
            break;
        default:
            break;
    }

    return os;
}
*/


