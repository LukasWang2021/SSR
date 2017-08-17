/*************************************************************************
	> File Name: parameter_manager_param_group.cpp
	> Author: Feng Yun
	> Mail: yun.feng@foresight-robotics.com
	> Created Time: 2016年11月07日 星期一 17时12分37秒
 ************************************************************************/

#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_error_code.h>
#include <boost/filesystem.hpp>
#include <memory>
#include <fstream>
#include <sstream>

#define LOG_BUFFER_SIZE 256

using std::cout;
using std::endl;
using std::stringstream;

using std::string;
using std::vector;
using std::map;

using XmlRpc::XmlRpcValue;

namespace fst_parameter {

void ParamGroup::test(void) {

    cout << "----------------Test begin: ---------------" << endl;
    cout << value_ << endl;
    string test = value_.toXml();
    cout << test << endl;
    value_.clear();
    cout << "clear:" << value_ << endl;

    value_.fromXml(test);
    cout << "rebuild from xml:" << endl << value_ << endl;



    cout << "----------------Test end ---------------" << endl;
/*
    cout << "----------------Test begin: ---------------" << endl;
    YAML::Node pnode = *root_;
    cout << pnode["name"]["first"]["sck"] << endl;
    

    YAML::Node node1 = (*root_)["name"];
    YAML::Node node2 = node1["first"];
    YAML::Node node3 = node2["sck"];
    cout << node3.Scalar() << endl;
    cout << "remove: " << (*root_)["name"]["first"].remove("sdd") << endl;
    cout << node3.Scalar() << endl;

    vector<YAML::Node> v_node;
    vector<string> v_key;
    v_key.push_back("name");
    v_key.push_back("first");
    v_key.push_back("sck");
    v_node.resize(5);
    v_node[0] = (*root_)[v_key[0]];
    for (int i=0; i<2; ++i) { 
        v_node[i+1] = v_node[i][v_key[i+1]];
    }
    
    cout << "here: " << v_node[2].Scalar() << endl;
    v_node[2] = 5.8;
    cout << "here: " << v_node[2].Scalar() << endl;

    // pnode["name"]["second"] = "sdfssdf";
    // pnode = pnode["first"];
    // pnode = pnode["sck"];

    // (*root_)["name"]["first"]["sck"] = 5.45;
    cout << (*root_)["name"]["first"]["sck"].Scalar() << endl;
    cout << YAML::Dump(*root_);
    cout << "----------------Test end ---------------" << endl;
*/
}


ParamGroup::ParamGroup(const string &file, const string &ns) {
    clearLastError();
    
    if (!file.empty()) {
        loadParamFile(file);
    }

    namespace_ = ns;
}

ParamGroup::~ParamGroup() {
}

bool ParamGroup::deleteParam(const string &key) {
    if (key.empty()) {
        value_.clear();
        return true;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    
    for (it = cooked_key.begin(); it != cooked_key.end() - 1; ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->delMember(*it)) {
        return true;
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
}

const ErrorCode& ParamGroup::getLastError(void) {
    return last_error_;
}

void ParamGroup::clearLastError(void) {
    last_error_ = SUCCESS;
}

const string ParamGroup::getNamespace(void) {
    return namespace_;
}

void ParamGroup::setNamespace(const string &ns) {
    namespace_ = ns;
}

bool ParamGroup::getParam(const string &key, bool &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isBool()) {
        value = (bool)*pv;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, int &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    
    if (pv->isInt()) {
        value = (int)*pv;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, double &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isDouble()) {
        value = (double)(*pv);
        return true;
    }
    else if (pv->isInt()) {
        value = 1.0 * (int)(*pv);
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, string &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isString()) {
        value = (string)*pv;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<bool> &value) {
    value.clear();
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isArray()) {
        int length = pv->size();
        for (int cnt = 0; cnt < length; ++cnt) {
            if ((*pv)[cnt].isBool()) {
                value.push_back((bool)((*pv)[cnt]));
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<int> &value) {
    value.clear();
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isArray()) {
        int length = pv->size();
        for (int cnt = 0; cnt < length; ++cnt) {
            if ((*pv)[cnt].isInt()) {
                value.push_back((int)((*pv)[cnt]));
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<double> &value) {
    value.clear();
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isArray()) {
        int length = pv->size();
        for (int cnt = 0; cnt < length; ++cnt) {
            if ((*pv)[cnt].isDouble()) {
                value.push_back((double)((*pv)[cnt]));
            }
            else if ((*pv)[cnt].isInt()) {
                value.push_back(1.0 * (int)((*pv)[cnt]));
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<string> &value) {
    value.clear();
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    if (pv->isArray()) {
        int length = pv->size();
        for (int cnt = 0; cnt < length; ++cnt) {
            if ((*pv)[cnt].isString()) {
                value.push_back((string)((*pv)[cnt]));
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, ParamValue &value) {
    value.clear();
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }

    value = *pv;
    return true;
}

bool ParamGroup::getNodeFromParamValue(ParamValue &value, YAML::Node &node) {
    node.reset();
    if (value.isStruct()) {
        bool result = true;
        for (ParamValue::iterator it = value.begin(); it != value.end(); ++it) {
            YAML::Node temp_node;
            if (getNodeFromParamValue(it->second, temp_node) == true) {
                node[it->first] = temp_node;
            }
            else {
                result = false;
                break;
            }
        }
        return result;
    }
    else if (value.isArray()) {
        bool result = true;
        int length = value.size();
        for (int cnt = 0; cnt < length; ++cnt) {
            YAML::Node temp_node;
            if (getNodeFromParamValue(value[cnt], temp_node) == true) {
                node.push_back(temp_node);
            }
            else {
                result = false;
                break;
            }
        }
        return result;
    }
    else if (value.isScalar()) {
        switch(value.getType()) {
            case ParamValue::TYPEBOOL:      node = (bool)value;     return true;
            case ParamValue::TYPEINT:       node = (int)value;      return true;
            case ParamValue::TYPEDOUBLE:    node = (double)value;   return true;
            case ParamValue::TYPESTRING:    node = (string)value;   return true;
            default:    return false;
        }
    }
    else {
        return false;
    }
}

bool ParamGroup::getParamValueFromNode(YAML::Node &node, ParamValue &value) {
    value.clear();
    if (node.IsMap()) {
        bool result = true;
        for (YAML::iterator it = node.begin(); it != node.end(); ++it) {
            ParamValue temp_value;
            if (getParamValueFromNode(it->second, temp_value) == true) {
                value[it->first.Scalar()] = temp_value;
            }
            else {
                result = false;
                break;
            }
        }
        return result;
    }
    else if (node.IsSequence()) {
        int  length = node.size();
        value.setSize(length);
        for (int cnt = 0; cnt < node.size(); ++cnt) {
            YAML::Node temp_node = node[cnt];
            if (getParamValueFromNode(temp_node, value[cnt]) != true) {
                return false;
            }
        }
        return true;
    }
    else if(node.IsScalar()) {
        return getParamValueFromString(node.Scalar(), value);
    }
    else {
        return false;
    }
}

bool ParamGroup::hasParam(const string &key) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }     
    }
    return true;
}

bool ParamGroup::dumpParamFile(const string &file) {
    time_t begin, end;
    begin = clock();
    info("Dumping YAML file ... ");
    string config_file;
    if (file[0] != '/') {
        char temp[1024] = {0};
        int length = readlink("/proc/self/exe", temp, sizeof(temp));
        if (length > 0 && length < sizeof(temp)) {
            boost::filesystem::path executable(temp);
            config_file = executable.parent_path().parent_path().parent_path().string() + "/" + file;
            info(config_file.c_str());
        }
        else {
            last_error_ = BAD_FILE_PATH;
            return false;
        }
    }
    else {
        config_file = file;
    }

    YAML::Node root;
    string param_string;
    if (!getNodeFromParamValue(value_, root)) {
        error("Failed to reform ParamValue -> YAML::Node !");
        last_error_ = FAIL_DUMPING_PARAM;
        return false;
    }
    param_string = YAML::Dump(root);
    param_string = param_string + "\n#END";

    std::ofstream yaml_handle(config_file.c_str());
    if (!yaml_handle.is_open()) {
        error("Fail to open %s", config_file.c_str());
        last_error_ = FAIL_OPENNING_FILE;
        return false;
    }
    yaml_handle << param_string;
    yaml_handle.close();

    string backup_file = config_file + ".backup";
    std::ofstream backup_handle(backup_file.c_str());
    if (!backup_handle.is_open()) {
        error("Fail to open %s", backup_file.c_str());
        last_error_ = FAIL_OPENNING_FILE;
        return false;
    }
    backup_handle << param_string;
    backup_handle.close();

    end = clock();
    info("Success! runtime = %f", double(end - begin) / CLOCKS_PER_SEC);
    return true;
}

bool ParamGroup::loadParamFile(const string &file) {
    time_t begin, end;
    begin = clock();
    value_.clear();
    info("Loading YAML file ... ");
    if (file.size() < 6) {
        error("Error: bad name");
        last_error_ = BAD_FILE_NAME;
        return false;
    }
    string extension = file.substr(file.size() - 5);
    vector<string> path_vector;
    split(file.substr(0, file.size() - 5), path_vector);
    string file_name = path_vector.back();
    if (extension != ".yaml" && extension != ".YAML" && extension != ".Yaml") {
        error("Error: Cannot open a non YAML file");
        last_error_ = BAD_FILE_EXTENSION;
        return false;
    }
    
    string yaml_file;
    if (file[0] != '/') {
        char temp[1024] = {0};
        int length = readlink("/proc/self/exe", temp, sizeof(temp));
        if (length > 0 && length < sizeof(temp)) {
            boost::filesystem::path executable(temp);
            yaml_file = executable.parent_path().parent_path().parent_path().string() + "/" + file;
            info(yaml_file.c_str());
        }
        else {
            error("Error: bad path");
            last_error_ = BAD_FILE_PATH;
            return false;
        }
    }
    else {
        yaml_file = file;
        info(yaml_file.c_str());
    }
    
    bool is_yaml_valid = false;
    string yaml_str;
    std::ifstream yaml_handle_r(yaml_file.c_str());
    if (yaml_handle_r.is_open()) {
        string temp_str((std::istreambuf_iterator<char>(yaml_handle_r)),
                         std::istreambuf_iterator<char>());
        yaml_str = temp_str;
        yaml_handle_r.close();
        size_t pos = yaml_str.find_last_of("#");
        if (pos != string::npos && yaml_str.substr(pos, 4) == "#END") {
            try {
                YAML::Node root = YAML::Load(yaml_str);
                if (getParamValueFromNode(root, value_))
                    is_yaml_valid = true;
            }
            catch (YAML::Exception& exception) {
                warn(exception.what());
                is_yaml_valid = false;
                last_error_ = FAIL_BUILDING_PARAM_TREE;
            }
        }
        else {
            is_yaml_valid = false;
            last_error_ = FAIL_BUILDING_PARAM_TREE;
        }
    }
    else {
        is_yaml_valid = false;
        last_error_ = FAIL_OPENNING_FILE;
    }

    if (is_yaml_valid == true) {
        string back_file = yaml_file + ".backup";
        std::ifstream back_handle_r(back_file.c_str());
        if (back_handle_r.is_open()) {
            string back_str((std::istreambuf_iterator<char>(back_handle_r)),
                             std::istreambuf_iterator<char>());
            back_handle_r.close();
            if (back_str == yaml_str) {
                end = clock();
                info("Success! runtime = %f", double(end - begin) / CLOCKS_PER_SEC);
                return true;
            }
            else {
                warn("Mismatch detected, updating backup file");
                std::ofstream back_handle_w(back_file.c_str());
                if (back_handle_w.is_open()) {
                    back_handle_w << yaml_str;
                    back_handle_w.close();
                    end = clock();
                    info("Success! runtime = %f", double(end - begin) / CLOCKS_PER_SEC);
                    return true;
                }
                else {
                    error("Updata backup file failed, loadParamFile abort");
                    last_error_ = FAIL_OPENNING_FILE;
                    return false;
                }
            }
        }
        else {
            warn("Cannot read backup file, re-backing up ...");
            std::ofstream back_handle_w(back_file.c_str());
            if (back_handle_w.is_open()) {
                back_handle_w << yaml_str;
                back_handle_w.close();
                end = clock();
                info("Success! runtime = %f", double(end - begin) / CLOCKS_PER_SEC);
                return true;
            }
            else {
                error("Re-back up failed, loadParamFile abort");
                last_error_ = FAIL_OPENNING_FILE;
                return false;
            }
        }
    }
    else {  
        warn("Cannot build parameter tree from YAML file, restoring from backup file ... ");
        bool is_back_valid = false;
        string back_str;
        string back_file = yaml_file + ".backup";
        std::ifstream back_handle_r(back_file.c_str());
        if (back_handle_r.is_open()) {
            string temp_str((std::istreambuf_iterator<char>(back_handle_r)),
                             std::istreambuf_iterator<char>());
            back_str = temp_str;
            back_handle_r.close();
            if (back_str.substr(back_str.length() - 5, 4) == "#END") {
                try {
                    YAML::Node root = YAML::Load(back_str);
                    if (getParamValueFromNode(root, value_))
                        is_back_valid = true;
                }
                catch (YAML::Exception& exception) {
                    warn(exception.what());
                    last_error_ = FAIL_BUILDING_PARAM_TREE;
                    is_back_valid = false;
                }
            }
            else {
                is_back_valid = false;
                last_error_ = FAIL_BUILDING_PARAM_TREE;
            }
        }
        else {
            warn("Cannot open backup file");
            last_error_ = FAIL_OPENNING_FILE;
            is_back_valid = false;
        }
        if (is_back_valid == true) {
            std::ofstream yaml_handle_w(yaml_file.c_str());
            if (yaml_handle_w.is_open()) {
                yaml_handle_w << back_str;
                yaml_handle_w.close();
                end = clock();
                info("Success! runtime = %f", double(end - begin) / CLOCKS_PER_SEC);
                return true;
            }
            else {
                error("Cannot restore YAML from backup file, loadParamFile abort");
                last_error_ = FAIL_OPENNING_FILE;
                return false;
            }           
        }
        else {
            error("Cannot build parameter tree from backup file, loadParamFile abort");
            return false;
        }
    }
}

bool ParamGroup::setParam(const string &key, bool value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isBool()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, int value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isInt()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, double value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isDouble()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const string &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isString()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const char *value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isString()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const vector<bool> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isArray()) {
        int length = value.size();
        pv->clear();
        pv->setSize(length);
        for (int cnt = 0; cnt < length; ++cnt) {
            (*pv)[cnt] = value[cnt];
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const vector<int> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isArray()) {
        int length = value.size();
        pv->clear();
        pv->setSize(length);
        for (int cnt = 0; cnt < length; ++cnt) {
            (*pv)[cnt] = value[cnt];
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const vector<double> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isArray()) {
        int length = value.size();
        pv->clear();
        pv->setSize(length);
        for (int cnt = 0; cnt < length; ++cnt) {
            (*pv)[cnt] = value[cnt];
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const vector<string> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;
    
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isArray()) {
        int length = value.size();
        pv->clear();
        pv->setSize(length);
        for (int cnt = 0; cnt < length; ++cnt) {
            (*pv)[cnt] = value[cnt];
        }
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const ParamValue &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<string>::iterator it;
    ParamValue *pv = &value_;

    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (pv->hasMember(*it)) {
            pv = &(*pv)[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }

    if (pv->isStruct()) {
        *pv = value;
        return true;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}
/*
bool ParamGroup::getRemoteParam(const string &key, XmlRpcValue &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }
    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, int &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, double &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, bool &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, string &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, vector<int> &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, vector<double> &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParam(const string &key, map<string, XmlRpcValue> &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, XmlRpcValue &value) {
    if (key.empty())
        return false;
    else if (key[0] != '/')
        return false;

    return ros::param::get(key, value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, int &value) {
    XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpcValue::TypeInt) {
            value = (int)temp;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool ParamGroup::getRemoteParamImpl(const string &key, double &value) {
    XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpcValue::TypeDouble) {
            value = (double)temp;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool ParamGroup::getRemoteParamImpl(const string &key, bool &value) {
    XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpcValue::TypeBoolean) {
            value = (bool)temp;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool ParamGroup::getRemoteParamImpl(const string &key, string &value) {
    XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpcValue::TypeString) {
            value = (string)temp;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool ParamGroup::getRemoteParamImpl(const string &key, vector<int> &value) {
    return ros::param::get(key, value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, vector<double> &value) {
    return ros::param::get(key, value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, map<string, XmlRpcValue> &value) {
    XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpcValue::TypeStruct) {
            value.clear();
            for (XmlRpcValue::iterator it = temp.begin(); it != temp.end(); ++it) {
                value.insert(std::pair<string, XmlRpcValue>(it->first, it->second));
            }
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }

}

bool ParamGroup::uploadParam() {
    if (!is_initialized_) {
        if (ros::isInitialized()) {
            is_initialized_ = true;
        }
        else {
            last_error_ = COMMUNICATION_ERROR;
            return false;
        }
    }
    if (!root_->IsNull()) {
        ros::param::del(getNamespace());
        return uploadParam(*root_, getNamespace());
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
}

bool ParamGroup::uploadParam(const YAML::Node &node, const string &path) {
    // cout << "uploadParam: " << path << " ";
    if (node.IsScalar()) {
        // cout << "is scalar" << endl;
        XmlRpcValue value;
        parseScalar(node.Scalar(), value);
        ros::param::set(path, value);
        return true;
    }
    else if (node.IsMap()) {
        // cout << "is map" << endl;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            string sub_path = path + "/" + it->first.as<string>();
            if (!uploadParam(it->second, sub_path)) {
                last_error_ = COMMUNICATION_ERROR;
                return false;
            }
        }
        return true;
    }
    else if (node.IsSequence()) {
        // cout << "is sequence" << endl;
        XmlRpcValue value;
        XmlRpcValue array;
        array.setSize(node.size());
        int cnt = 0;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            if (!it->IsScalar()) {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
            if (parseScalar(it->Scalar(), value)) {
                array[cnt] = value;
                ++cnt;
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        ros::param::set(path, array);
        return true;
    }  // else if (node.IsSequence())
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}
*/

bool ParamGroup::getParamValueFromString(const string &scalar, ParamValue &value) {
    if (scalar.empty()) {
        value.clear();
        return true;
    }

    stringstream ss(scalar);
    int int_value;
    ss >> int_value;
    if (ss.eof()) {
        value = int_value;
        return true;
    }
    
    double double_value;
    ss.str("");
    ss.clear();
    ss << scalar;
    ss >> double_value;
    if (ss.eof()) {
        value = double_value;
        return true;
    }
            
    if (scalar == "true" || scalar == "True" || scalar == "TRUE") {
        value = true;
        return true;
    }
    else if (scalar == "false" || scalar == "False" || scalar == "FALSE") {
        value = false;
        return true;
    }
    
    value = scalar;
    return true;
}

void ParamGroup::split(const string &raw, vector<string> &cooked) {
    cooked.clear();
    if (raw.empty())
        return;
    
    string str(raw);
    resolve(str);
    
    string sep = "/";
    string temp;
    string::size_type begin = str.find_first_not_of(sep);
    string::size_type position = 0;

    while (begin != string::npos) {
        position = str.find(sep, begin);
        if (position != string::npos) {
            temp = str.substr(begin, position - begin);
            begin = position + sep.length();
        }
        else {
            temp = str.substr(begin);
            begin = position;
        }

        if (!temp.empty()) {
            cooked.push_back(temp);
            temp.clear();
        }
    }
    return;
}

void ParamGroup::resolve(string &str) {
    for (string::iterator it = str.begin(); it != str.end(); ++it) {
        if (*it == '.')
            *it = '/';
    }
}

void ParamGroup::info(const char *format, ...) {
    char buf[LOG_BUFFER_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, LOG_BUFFER_SIZE - len - 2, format, vp);
    va_end(vp);
    buf[len] = '\0';
    
    cout << buf << endl;
}

void ParamGroup::warn(const char *format, ...) {
    char buf[LOG_BUFFER_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ WARN][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, LOG_BUFFER_SIZE - len - 2, format, vp);
    va_end(vp);
    buf[len] = '\0';

    cout << "\033[33m" << buf << "\033[0m" << endl;
}

void ParamGroup::error(const char *format, ...) {
    char buf[LOG_BUFFER_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, LOG_BUFFER_SIZE - len - 2, format, vp);
    va_end(vp);
    buf[len + 1] = '\0';

    cout << "\033[31m" << buf << "\033[0m" << endl;
}


}
