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


using std::cout;
using std::endl;
using std::stringstream;

using std::string;
using std::vector;
using std::map;


namespace fst_parameter {
/*
void ParamGroup::test(void) {
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
}
*/

ParamGroup::ParamGroup(const string &file, const string &ns) {
    clearLastError();
    
    this->root_namespace_ = ns;
    this->root_ = new YAML::Node();
    if (this->root_ == NULL) {
        printError("Cannot initialize parametr tree.");
        this->last_error_ = PARAM_FAIL_IN_INIT;
        return;
    }

    if (!file.empty()) {
        if (!loadParamFile(file))
            return;
    }

    if (ros::isInitialized()) {
        this->is_initialized_ = true;
    }
    else {
        this->is_initialized_ = false;
        printError("Cannot use remote parameter server, need to execute ros::init() first");
    }
    return;
}

ParamGroup::~ParamGroup() {
    root_->reset();
    delete root_;
}

bool ParamGroup::deleteParam(const string &key) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size - 1; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size - 1) {
            ++key_itr;
            if ((*node_itr).remove(*key_itr)) {
                return true;
            }
            else {
                last_error_ = PARAM_NOT_FOUND;
                return false;
            }
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    /*
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end() - 1; ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }
    if ((it == cooked_key.end() - 1) && node[*it]) {
        return node.remove(*it);
    }
    else {
        return false;
    }*/
}

void ParamGroup::deleteParamTree(void) {
    root_->reset();
}

const ErrorCode& ParamGroup::getLastError(void) {
    return last_error_;
}

void ParamGroup::clearLastError(void) {
    last_error_ = SUCCESS;
}

const string ParamGroup::getNamespace(void) {
    return root_namespace_ + "/" + sub_namespace_;
}

bool ParamGroup::getParam(const string &key, int &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsScalar()) {
        if (judgeType(node.Scalar()) == e_type_int) {
            XmlRpc::XmlRpcValue v;
            parseScalar(node.Scalar(), v);
            value = (int)v;
            return true;
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if(it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
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
    //std::auto_ptr<YAML::Node> pnode = (*root_).Clone();
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsScalar()) {
        if (judgeType(node.Scalar()) == e_type_double) {
            XmlRpc::XmlRpcValue v;
            parseScalar(node.Scalar(), v);
            value = (double)v;
            return true;
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if(it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, bool &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsScalar()) {
        if (judgeType(node.Scalar()) == e_type_bool) {
            XmlRpc::XmlRpcValue v;
            parseScalar(node.Scalar(), v);
            value = (bool)v;
            return true;
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if(it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
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
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsScalar()) {
        if (judgeType(node.Scalar()) == e_type_string) {
            XmlRpc::XmlRpcValue v;
            parseScalar(node.Scalar(), v);
            value = (string)v;
            return true;
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if(it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<int> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsSequence()) {
        YAML::const_iterator itr = node.begin();
        if (judgeType(itr->Scalar()) == e_type_int) {
            XmlRpc::XmlRpcValue v;
            
            for (itr = node.begin(); itr != node.end(); ++itr) {
                if (judgeType(itr->Scalar()) != e_type_int)
                    break;
                parseScalar(itr->Scalar(), v);
                value.push_back((int)v);
            }  // for (itr = node.begin(); itr != node.end(); ++itr)
            if (itr == node.end()) {
                return true;
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if (it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<double> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsSequence()) {
        YAML::const_iterator itr = node.begin();
        if (judgeType(itr->Scalar()) == e_type_double) {
            XmlRpc::XmlRpcValue v;
            
            for (itr = node.begin(); itr != node.end(); ++itr) {
                if (judgeType(itr->Scalar()) != e_type_double) {
                    break;
                }
                parseScalar(itr->Scalar(), v);
                if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    value.push_back((double)v);
                else if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    value.push_back((double)((int)v));
                else
                    break;
            }  // for (itr = node.begin(); itr != node.end(); ++itr)
            if (itr == node.end()) {
                return true;
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if (it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, vector<string> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsSequence()) {
        YAML::const_iterator itr = node.begin();
        if (judgeType(itr->Scalar()) == e_type_string) {
            XmlRpc::XmlRpcValue v;
            
            for (itr = node.begin(); itr != node.end(); ++itr) {
                if (judgeType(itr->Scalar()) != e_type_string) {
                    break;
                }
                parseScalar(itr->Scalar(), v);
                if (v.getType() == XmlRpc::XmlRpcValue::TypeString)
                    value.push_back((string)v);
                else
                    break;
            }  // for (itr = node.begin(); itr != node.end(); ++itr)
            if (itr == node.end()) {
                return true;
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
    }
    else if (it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, map<string, XmlRpc::XmlRpcValue> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsMap()) {
        value.clear();
        YAML::const_iterator itr = node.begin();
        XmlRpc::XmlRpcValue v;
        for (; itr != node.end(); ++itr) {
            try {
                parseScalar(itr->second.Scalar(), v);
                value.insert(std::pair<string, XmlRpc::XmlRpcValue>(itr->first.as<string>(), v));
            }
            catch (YAML::Exception& exception) {
                cout << exception.what() << endl;
                last_error_ = PARSE_ERROR;
                return false;
            }
        }
        return true;
    }
    else if (it != cooked_key.end()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

bool ParamGroup::getParam(const string &key, XmlRpc::XmlRpcValue &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
    
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end()) {
        value.clear();
        getXmlRpcValueFromNode(node, value);
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
}

bool ParamGroup::getXmlRpcValueFromNode(YAML::Node node, XmlRpc::XmlRpcValue &value) {
    value.clear();
    if (node.IsMap()) {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            XmlRpc::XmlRpcValue temp_value;
            getXmlRpcValueFromNode(it->second, temp_value);
            value[it->first.Scalar()] = temp_value;
        }
    }
    else if (node.IsSequence()) {
        int cnt = 0;
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            XmlRpc::XmlRpcValue temp_value;
            getXmlRpcValueFromNode(*it, temp_value);
            value[cnt] = temp_value;
            cnt++;
        }
    }
    else if(node.IsScalar()) {
        parseScalar(node.Scalar(), value);
        return true;
    }
}

bool ParamGroup::hasParam(const string &key) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            break;
        }
    }

    return it == cooked_key.end() ? true : false;
}

/*
bool ParamGroup::getXmlRpcValue(const string &key, XmlRpc::XmlRpcValue &value) {
    if (key.empty())
        return false;

    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = *root_;
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it!=cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsScalar()) {
        XmlRpc::XmlRpcValue v;
        parseScalar(node.Scalar(), v);
        return true;
    }
    else {
        return false;
    }
}
*/

bool ParamGroup::dumpParamFile(const string &file) {
    printInfo("Dumping YAML file ... ");

    string config_file;
    if (file[0] != '/') {
        char temp[1024] = {0};
        int length = readlink("/proc/self/exe", temp, sizeof(temp));
        if (length > 0 && length < sizeof(temp)) {
            boost::filesystem::path executable(temp);
            config_file = executable.parent_path().parent_path().parent_path().string() + "/" + file;
            printInfo(config_file.c_str());
        }
        else {
            last_error_ = BAD_FILE_PATH;
            return false;
        }
    }
    else {
        config_file = file;
    }

    std::ofstream yaml_handle(config_file.c_str());
    if (!yaml_handle.is_open()) {
        printError("Fail!");
        last_error_ = FAIL_OPENNING_FILE;
        return false;
    }
    yaml_handle << YAML::Dump(*root_);
    yaml_handle << "\n#END";
    yaml_handle.close();

    string backup_file = config_file + ".backup";
    std::ofstream backup_handle(backup_file.c_str());
    if (!backup_handle.is_open()) {
        printError("Fail!");
        last_error_ = FAIL_OPENNING_FILE;
        return false;
    }
    backup_handle << YAML::Dump(*root_);
    yaml_handle << "\n#END";
    backup_handle.close();

    printInfo("Success!");
    return true;
}

/*
bool ParamGroup::loadParamFile(const string &file) {
    if (file.size() < 6) {
        printError("Bad file name");
        last_error_ = BAD_FILE_NAME;
        return false;
    }
    
    sub_namespace_ = "";
    if (!root_->IsNull()) {
        deleteParamTree();
    }

    string extension = file.substr(file.size() - 5);
    vector<string> path_vector;
    split(file.substr(0, file.size() - 5), path_vector);
    string file_name = path_vector.back();
    if (extension != ".yaml" && extension != ".YAML" && extension != ".Yaml") {
        printError("Error: Cannot open a non YAML file!");
        last_error_ = BAD_FILE_EXTENSION;
        return false;
    }
    
    printInfo("Loading YAML file ... ");

    bool is_yaml_valid = true;
    bool is_backup_valid = true;

    string config_file;
    if (file[0] != '/') {
        char temp[1024] = {0};
        int length = readlink("/proc/self/exe", temp, sizeof(temp));
        if (length > 0 && length < sizeof(temp)) {
            boost::filesystem::path executable(temp);
            config_file = executable.parent_path().parent_path().parent_path().string() + "/" + file;
            printInfo(config_file.c_str());
        }
        else {
            last_error_ = BAD_FILE_PATH;
            return false;
        }
    }
    else {
        config_file = file;
    }

    std::ifstream yaml_handle(config_file.c_str());
    if (!yaml_handle.is_open()) {
        last_error_ = FAIL_OPENNING_FILE;
        is_yaml_valid = false;
    }
    
    string backup_file = config_file + ".backup";
    std::ifstream backup_handle(backup_file.c_str());
    if (!backup_handle.is_open()) {
        last_error_ = FAIL_OPENNING_FILE;
        is_backup_valid = false;
    }
    
    if (is_yaml_valid && is_backup_valid) {
        string yaml_str((std::istreambuf_iterator<char>(yaml_handle)),
                         std::istreambuf_iterator<char>());
        string backup_str((std::istreambuf_iterator<char>(backup_handle)),
                           std::istreambuf_iterator<char>());
        if (yaml_str == backup_str) {
            if (yaml_str.substr(yaml_str.length() - 5) != "/n#END") {
                last_error_ = FAIL_BUILDING_PARAM_TREE;
                return false;
            }
            try {
                *root_ = YAML::Load(yaml_str);
            }
            catch (YAML::Exception& exception) {
                printError(exception.what());
                last_error_ = FAIL_BUILDING_PARAM_TREE;
                return false;
            }
            sub_namespace_ = file_name;
            printInfo("Success!");
            return true;
        }
        else {
            printInfo("Mismatch detected!");
            try {
                *root_ = YAML::Load(yaml_str);
            }
            catch (YAML::Exception& exception) {
                // cout << endl << exception.what() << endl;
                printInfo("Restoring YAML from backup file ... ");
                try {
                    *root_ = YAML::Load(backup_str);
                }
                catch (YAML::Exception& exception) {
                    // cout << endl << exception.what() << endl;
                    printError("Failed");
                    last_error_ = FAIL_BUILDING_PARAM_TREE;
                    return false;
                }
                yaml_handle.close();
                std::ofstream yaml_other_handle(config_file.c_str());
                if (!yaml_other_handle.is_open()) {
                    printError("Failed");
                    last_error_ = FAIL_RESTORING_YAML;
                    return false;
                }
                yaml_other_handle << backup_str;
                yaml_other_handle.close();
                sub_namespace_ = file_name;
                printInfo("Success!");
                return true;
            }
            printInfo("Updating backup file ... ");
            backup_handle.close();
            std::ofstream backup_other_handle(backup_file.c_str());
            if (!backup_other_handle.is_open()) {
                printError("Failed!");
                last_error_ = FAIL_UPDATING_BACKUP;
                return false;
            }
            backup_other_handle << yaml_str;
            backup_other_handle.close();
            sub_namespace_ = file_name;
            printInfo("Success!");
            return true;
        }
    }
    else if (is_yaml_valid && !is_backup_valid) {
        printInfo("Mismatch detected! Updating backup file ... ");
        string yaml_str((std::istreambuf_iterator<char>(yaml_handle)),
                         std::istreambuf_iterator<char>());
        try {
            *root_ = YAML::Load(yaml_str);
        }
        catch (YAML::Exception& exception) {
            // cout << endl << exception.what() << endl;
            printError("Failed!");
            last_error_ = FAIL_BUILDING_PARAM_TREE;
            return false;
        }
        backup_handle.close();
        std::ofstream backup_other_handle(backup_file.c_str());
        if (!backup_other_handle.is_open()) {
            printError("Failed!");
            last_error_ = FAIL_UPDATING_BACKUP;
            return false;
        }
        backup_other_handle << yaml_str;
        backup_other_handle.close();
        sub_namespace_ = file_name;
        printInfo("Success!");
        return true;
    }
    else if (!is_yaml_valid && is_backup_valid) {
        printInfo("Mismatch detected! Restoring YAML from backup file ... ");
        string backup_str((std::istreambuf_iterator<char>(backup_handle)),
                         std::istreambuf_iterator<char>());
        try {
            *root_ = YAML::Load(backup_str);
        }
        catch (YAML::Exception& exception) {
            // cout << endl << exception.what() << endl;
            printError("Failed!");
            last_error_ = FAIL_BUILDING_PARAM_TREE;
            return false;
        }
        yaml_handle.close();
        std::ofstream yaml_other_handle(config_file.c_str());
        if (!yaml_other_handle.is_open()) {
            printError("Failed!");
            last_error_ = FAIL_RESTORING_YAML;
            return false;
        }
        yaml_other_handle << backup_str;
        yaml_other_handle.close();
        sub_namespace_ = file_name;
        printInfo("Success!");
        return true;
    }
    else {
        printError("Bad YAML file! What the worse: we cannot restore the YAML file from backups!");
        last_error_ = FAIL_BUILDING_PARAM_TREE;
        return false;
    }
}
*/

bool ParamGroup::loadParamFile(const string &file) {
    sub_namespace_ = "";
    if (!root_->IsNull()) {
        deleteParamTree();
    }

    printInfo("Loading YAML file ... ");
    if (file.size() < 6) {
        printError("Error: bad name");
        last_error_ = BAD_FILE_NAME;
        return false;
    }
    string extension = file.substr(file.size() - 5);
    vector<string> path_vector;
    split(file.substr(0, file.size() - 5), path_vector);
    string file_name = path_vector.back();
    if (extension != ".yaml" && extension != ".YAML" && extension != ".Yaml") {
        printError("Error: Cannot open a non YAML file");
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
            printInfo(yaml_file.c_str());
        }
        else {
            printError("Error: bad path");
            last_error_ = BAD_FILE_PATH;
            return false;
        }
    }
    else {
        yaml_file = file;
        printInfo(yaml_file.c_str());
    }
    
    bool is_yaml_valid = false;
    string yaml_str;
    std::ifstream yaml_handle_r(yaml_file.c_str());
    if (yaml_handle_r.is_open()) {
        string temp_str((std::istreambuf_iterator<char>(yaml_handle_r)),
                         std::istreambuf_iterator<char>());
        yaml_str = temp_str;
        yaml_handle_r.close();
        if (yaml_str.substr(yaml_str.length() - 5, 4) == "#END") {
            try {
                *root_ = YAML::Load(yaml_str);
                is_yaml_valid = true;
            }
            catch (YAML::Exception& exception) {
                printError(exception.what());
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
                sub_namespace_ = file_name;
                printInfo("Success!");
                return true;
            }
            else {
                printInfo("Mismatch detected, updating backup file");
                std::ofstream back_handle_w(back_file.c_str());
                if (back_handle_w.is_open()) {
                    back_handle_w << yaml_str;
                    back_handle_w.close();
                    sub_namespace_ = file_name;
                    printInfo("Success!");
                    return true;
                }
                else {
                    printError("Updata backup file failed, loadParamFile abort");
                    last_error_ = FAIL_OPENNING_FILE;
                    return false;
                }
            }
        }
        else {
            printError("Cannot read backup file, re-backing up ...");
            std::ofstream back_handle_w(back_file.c_str());
            if (back_handle_w.is_open()) {
                back_handle_w << yaml_str;
                back_handle_w.close();
                sub_namespace_ = file_name;
                printInfo("Success!");
                return true;
            }
            else {
                printError("Re-back up failed, loadParamFile abort");
                last_error_ = FAIL_OPENNING_FILE;
                return false;
            }
        }
    }
    else {  
        printInfo("Cannot build parameter tree from YAML file, restoring from backup file ... ");
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
                    *root_ = YAML::Load(back_str);
                    is_back_valid = true;
                }
                catch (YAML::Exception& exception) {
                    printError(exception.what());
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
            printError("Cannot open backup file");
            last_error_ = FAIL_OPENNING_FILE;
            is_back_valid = false;
        }
        if (is_back_valid == true) {
            std::ofstream yaml_handle_w(yaml_file.c_str());
            if (yaml_handle_w.is_open()) {
                yaml_handle_w << back_str;
                yaml_handle_w.close();
                sub_namespace_ = file_name;
                printInfo("Success!");
                return true;
            }
            else {
                printError("Cannot restore YAML from backup file, loadParamFile abort");
                last_error_ = FAIL_OPENNING_FILE;
                return false;
            }           
        }
        else {
            printError("Cannot build parameter tree from backup file, loadParamFile abort");
            return false;
        }
    }
}

bool ParamGroup::setParam(const string &key, int value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
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
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, bool value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value ? "True" : "False";
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
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
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
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
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
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
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            // cout << (*root_)["name"]["first"]["sdc"].IsSequence() << endl;
            // cout << (*root_)["name"]["first"]["sdc"][0].as<double>() << endl;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
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
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
            // cout << (*root_)["name"]["first"]["sdc"].IsSequence() << endl;
            // cout << (*root_)["name"]["first"]["sdc"][0].as<double>() << endl;
            return true;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
}

bool ParamGroup::setParam(const string &key, const map<string, XmlRpc::XmlRpcValue> &value) {
    if (key.empty()) {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }
   
    vector<string> cooked_key;
    split(key, cooked_key);
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            XmlRpc::XmlRpcValue v;
            bool has_error = false;
            for (map<string, XmlRpc::XmlRpcValue>::const_iterator map_itr = value.begin();
                        map_itr != value.end(); ++map_itr) {
                switch (map_itr->second.getType()) {
                     case XmlRpc::XmlRpcValue::TypeInt:
                        v = map_itr->second;
                        (*node_itr)[map_itr->first] = (int)v;
                        break;
                     case XmlRpc::XmlRpcValue::TypeDouble:
                        v = map_itr->second;
                        (*node_itr)[map_itr->first] = (double)v;
                        break;
                     case XmlRpc::XmlRpcValue::TypeString:
                        v = map_itr->second;
                        (*node_itr)[map_itr->first] = (string)v;
                        break;
                     case XmlRpc::XmlRpcValue::TypeBoolean:
                        v = map_itr->second;
                        (*node_itr)[map_itr->first] = (bool)v ? "True" : "False";
                        break;
                    default:
                        has_error = true;
                        break;
                 }
                 if (has_error) {
                    break;
                 }
            }
            if (!has_error) {
                return true;
            }
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
            // cout << (*root_)["name"]["first"]["sdc"].IsSequence() << endl;
            // cout << (*root_)["name"]["first"]["sdc"][0].as<double>() << endl;
        }
        else {
            last_error_ = PARAM_NOT_FOUND;
            return false;
        }
    }
    else {
        last_error_ = PARAM_NOT_FOUND;
        return false;
    }

/*
    vector<string> cooked_key;
    split(key, cooked_key);
    YAML::Node node = YAML::Clone(*root_);
    vector<string>::iterator it;
    for (it = cooked_key.begin(); it != cooked_key.end(); ++it) {
        if (node.IsMap() && node[*it]) {
            node = node[*it];
        }
        else {
            break;
        }
    }

    if (it == cooked_key.end() && node.IsMap()) {
        value.clear();
        YAML::const_iterator itr = node.begin();
        XmlRpc::XmlRpcValue v;
        for (; itr != node.end(); ++itr) {
            try {
                parseScalar(itr->second.Scalar(), v);
                value.insert(std::pair<string, XmlRpc::XmlRpcValue>(itr->first.as<string>(), v));
            }
            catch (YAML::Exception& exception) {
                cout << exception.what() << endl;
                return false;
            }
        }
        return true;
    }
    else {
        return false;
    }*/
}

/*
template<typename T>
bool ParamGroup::setParameter(const string &key, const T &value) {
    if (key.empty())
        return false;

    vector<string> cooked_key;
    split(key, cooked_key);
    vector<YAML::Node> v_node;
    int vector_size = cooked_key.size();
    v_node.resize(vector_size);
    vector<YAML::Node>::iterator node_itr = v_node.begin();
    vector<string>::iterator key_itr = cooked_key.begin();
    if ((*root_)[cooked_key.front()]) {
        v_node.front() = (*root_)[cooked_key.front()];
        int loop = 1;
        for (; loop < vector_size; ++loop) {
            ++key_itr;
            if ((*node_itr)[*key_itr]) {
                *(node_itr + 1) = (*node_itr)[*key_itr];
                ++node_itr;
            }
            else {
                break;
            }
        }
        if (loop == vector_size) {
            *node_itr = value;
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
*/

bool ParamGroup::getRemoteParam(const string &key, XmlRpc::XmlRpcValue &value) {
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

bool ParamGroup::getRemoteParam(const string &key, map<string, XmlRpc::XmlRpcValue> &value) {
    if (!is_initialized_) {
        return false;
    }
    if (root_namespace_ == "" || sub_namespace_ == ""){
        return false;
    }

    return getRemoteParamImpl(root_namespace_ + "/" + sub_namespace_ + "/" + resolve(key), value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, XmlRpc::XmlRpcValue &value) {
    if (key.empty())
        return false;
    else if (key[0] != '/')
        return false;

    return ros::param::get(key, value);
}

bool ParamGroup::getRemoteParamImpl(const string &key, int &value) {
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeInt) {
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
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
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
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
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
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeString) {
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
    /*
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            value.clear();
            int size = temp.size();
            for (int i = 0; i < size; ++i) {
                value.push_back(temp[i]);
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
    */
}

bool ParamGroup::getRemoteParamImpl(const string &key, vector<double> &value) {
    return ros::param::get(key, value);
    /*
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            value.clear();
            int size = temp.size();
            for (int i = 0; i < size; ++i) {
                cout << temp[i].getType() << endl;
                value.push_back(temp[i]);
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
    */
}

bool ParamGroup::getRemoteParamImpl(const string &key, map<string, XmlRpc::XmlRpcValue> &value) {
    XmlRpc::XmlRpcValue temp;
    if (getRemoteParamImpl(key, temp)) {
        if (temp.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            value.clear();
            for (XmlRpc::XmlRpcValue::iterator it = temp.begin(); it != temp.end(); ++it) {
                value.insert(std::pair<string, XmlRpc::XmlRpcValue>(it->first, it->second));
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


    /*
template <typename T>
bool ParamGroup::getParam(const string &key, T &value) {
    cout << (*root_)["arm_group/cycle_time"] << endl;
    return true;
}*/

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
        XmlRpc::XmlRpcValue value;
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
        XmlRpc::XmlRpcValue value;
        XmlRpc::XmlRpcValue array;
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
        /*
        YAML::const_iterator it = node.begin();
        if (it->IsScalar()) {
            ScalarType type = judgeType(it->Scalar());
            if (type == e_type_int) {
                vector<int> array;
                XmlRpc::XmlRpcValue value;
                for (it = node.begin(); it != node.end(); ++it) {
                    if (judgeType(it->Scalar()) != e_type_int)
                        break;
                    parseScalar(it->Scalar(), value);
                    array.push_back((int)value);
                }  // for (it = node.begin(); it != node.end(); ++it)
                if (it == node.end()) {
                    ros::param::set(path, array);
                    return true;
                }
                else {
                    last_error_ = PARAM_TYPE_ERROR;
                    return false;
                }
            }  // if (type == e_type_int)
            else if (type == e_type_double) {
                vector<double> array;
                XmlRpc::XmlRpcValue value;
                for (it = node.begin(); it != node.end(); ++it) {
                    if (judgeType(it->Scalar()) == e_type_double) {
                        parseScalar(it->Scalar(), value);
                    }
                    else if (judgeType(it->Scalar()) == e_type_int) {
                        XmlRpc::XmlRpcValue tmp;
                        parseScalar(it->Scalar(), tmp);
                        int val1 = (int)tmp;
                        double val2 = val1;
                        value = val2;
                    }
                    else
                        break;

                    array.push_back((double)value);
                }  // for (it = node.begin(); it != node.end(); ++it)
                if (it == node.end()) {
                    ros::param::set(path, array);
                    return true;
                }
                else {
                    last_error_ = PARAM_TYPE_ERROR;
                    return false;
                }
            }  // else if (type == e_type_double)
            else if (type == e_type_string) {
                vector<string> array;
                XmlRpc::XmlRpcValue value;
                for (it = node.begin(); it != node.end(); ++it) {
                    if (judgeType(it->Scalar()) != e_type_string)
                        break;
                    parseScalar(it->Scalar(), value);
                    array.push_back((string)value);
                }  // for (it = node.begin(); it != node.end(); ++it)
                if (it == node.end()) {
                    ros::param::set(path, array);
                    return true;
                }
                else {
                    last_error_ = PARAM_TYPE_ERROR;
                    return false;
                }
            }  // else if (type == e_type_string)
            else if (type == e_type_bool) {
                vector<bool> array;
                XmlRpc::XmlRpcValue value;
                for (it = node.begin(); it != node.end(); ++it) {
                    if (judgeType(it->Scalar()) != e_type_bool)
                        break;
                    parseScalar(it->Scalar(), value);
                    array.push_back((bool)value);
                }  // for (it = node.begin(); it != node.end(); ++it)
                if (it == node.end()) {
                    ros::param::set(path, array);
                    return true;
                }
                else {
                    last_error_ = PARAM_TYPE_ERROR;
                    return false;
                }
            }  // else if (type == e_type_bool)
            else {
                last_error_ = PARAM_TYPE_ERROR;
                return false;
            }
        }  // // if (it->IsScalar())
        else {
            last_error_ = PARAM_TYPE_ERROR;
            return false;
        }
        */
    }  // else if (node.IsSequence())
    else {
        last_error_ = PARAM_TYPE_ERROR;
        return false;
    }
}

/*
template <typename T>
bool ParamGroup::parseSequence(const YAML::Node &node, ScalarType type, vector<T> &array) {

}
*/

fst_parameter::ScalarType ParamGroup::judgeType(const string &scalar) {
    stringstream ss(scalar);
    int int_value;
    ss >> int_value;
    if (ss.eof()) {
        return e_type_int;
    }
    else {
        double double_value;
        ss.str("");
        ss.clear();
        ss << scalar;
        ss >> double_value;
        if (ss.eof()) {
            return e_type_double;
        }
        else {
            if (scalar == "true" || scalar == "True" || scalar == "TRUE" ||
                scalar == "false" || scalar == "False" || scalar == "FALSE")
                return e_type_bool;
            else
                return e_type_string;
        }
    }
}

bool ParamGroup::parseScalar(const string &scalar, XmlRpc::XmlRpcValue &value) {
    if (scalar.empty())
        return false;

    stringstream ss(scalar);
    int int_value;
    ss >> int_value;
    if (ss.eof()) {
        XmlRpc::XmlRpcValue v(int_value);
        value = v;
        return true;
    }
    else {
        double double_value;
        ss.str("");
        ss.clear();
        ss << scalar;
        ss >> double_value;
        if (ss.eof()) {
            XmlRpc::XmlRpcValue v(double_value);
            value = v;
            return true;
        }
        else {
            if (scalar == "true" || scalar == "True" || scalar == "TRUE") {
                XmlRpc::XmlRpcValue v(true);
                value = v;
                return true;
            }
            else if (scalar == "false" || scalar == "False" || scalar == "FALSE") {
                XmlRpc::XmlRpcValue v(false);
                value = v;
                return true;
            }
            else {
                XmlRpc::XmlRpcValue v(scalar);
                value = v;
                return true;
            }
        }
    }
}
/*
bool ParamGroup::setNamespace(const string &space) {
    if (space.empty())
        return false;
    
    root_namespace_ = space;
}

bool ParamGroup::setSubNamespace(const string &space) {
    if (space.empty())
        return false;
    
    sub_namespace_ = space;
}*/

void ParamGroup::split(const string &raw, vector<string> &cooked) {
    if (raw.empty())
        return;

    string str = resolve(raw);
    
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

    /*
    if (cooked.size() > 1)
        return;

    cooked.clear();
    sep = ".";
    begin = raw.find_first_not_of(sep);
    position = 0;

    while (begin != string::npos) {
        position = raw.find(sep, begin);
        if (position != string::npos) {
            temp = raw.substr(begin, position - begin);
            begin = position + sep.length();
        }
        else {
            temp = raw.substr(begin);
            begin = position;
        }

        if (!temp.empty()) {
            cooked.push_back(temp);
            temp.clear();
        }
    }
    */
}

string ParamGroup::resolve(const string &str) {
    string temp = str;
    for (string::iterator it = temp.begin(); it != temp.end(); ++it) {
        if (*it == '.')
            *it = '/';
    }
    return temp;
}

inline void ParamGroup::logMessage(const string &str) {
    /*
    m_log_file_content += str;
    if (m_log_file_content.size() > 4096) {
        std::ofstream log_file_handle(m_log_file_name.c_str(), ios::app);
        if (log_file_handle.is_open()) {
            log_file_handle << m_log_file_content;
            log_file_handle.close();
            m_log_file_content.clear();
        }
    }
    */
    cout << str;
}

inline void ParamGroup::printInfo(const char *str) {
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    char buf[128];
    sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);
    string temp(buf);
    temp = temp + str + '\n';
    logMessage(temp);
}

template<typename T>
inline void ParamGroup::printInfo(const char *str, T value) {
    char buf[128];
    sprintf(buf, str, value);
    printInfo(buf);
}

inline void ParamGroup::printError(const char *str) {
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    char buf[128];
    sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);
    string temp(buf);
    temp = temp + str + '\n';
    cout << "\033[31m";
    logMessage(temp);
    cout << "\033[0m";
}

inline void ParamGroup::printError(const char *str, const ErrorCode &err) {
    char buf[128];
    sprintf(buf, str, err);
    printError(buf);
}

}
