#include "yaml_help.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace base_space;

YamlHelp::YamlHelp()
{

}

YamlHelp::~YamlHelp()
{

}

bool YamlHelp::loadParamFile(const std::string file_path)
{
    if(!file_path.empty())
    {
        try
        {
            node_ = YAML::LoadFile(file_path);
            return !node_.IsNull();
        }
        catch(YAML::Exception& exception)
        {
            std::cout<<exception.what()<<std::endl;
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool YamlHelp::dumpParamFile(const std::string file_path)
{
    if(!file_path.empty())
    {        
        std::ofstream fout(file_path);
        if(!fout.is_open())
        {
            return false;
        }
        try
        {
            YAML::Emitter emitter_;
            emitter_<<node_;
            fout<<emitter_.c_str();
            fout.close();
            return true;
        }
        catch(YAML::Exception& exception)
        {
            std::cout<<exception.what()<<std::endl;
            fout.close();
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, bool& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<bool>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<bool>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<bool>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<bool>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, int& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<int>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<int>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<int>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<int>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, double& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<double>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<double>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<double>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<double>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, std::string& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<std::string>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<std::string>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<string>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<string>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, std::vector<bool>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<std::vector<bool>>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<std::vector<bool>>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<std::vector<bool>>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<std::vector<bool>>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, std::vector<int>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<std::vector<int>>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<std::vector<int>>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<std::vector<int>>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<std::vector<int>>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, std::vector<double>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<std::vector<double>>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<std::vector<double>>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<std::vector<double>>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<std::vector<double>>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, std::vector<std::string>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    value = node_[key_set[0]].as<std::vector<std::string>>();
                    return true;
                }
            case 2: 
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]].as<std::vector<std::string>>();
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]].as<std::vector<std::string>>();
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    value = node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].as<std::vector<std::string>>();
                    return true;
                }
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

bool YamlHelp::getParam(const std::string& key, int* value, size_t size)
{    
    vector<int> tmp;

    if (getParam(key, tmp))
    {
        if (tmp.size() < size) size = tmp.size();
        for (size_t i = 0; i < size; i++) value[i] = tmp[i];
        return true;
    }

    return false;
}

bool YamlHelp::getParam(const std::string& key, double* value, size_t size)
{
    vector<double> tmp;

    if (getParam(key, tmp))
    {
        if (tmp.size() < size) size = tmp.size();
        for (size_t i = 0; i < size; i++) value[i] = tmp[i];
        return true;
    }
    return false;
}

   
bool YamlHelp::setParam(const std::string& key, const bool& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }                
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }    
}

bool YamlHelp::setParam(const std::string& key, const int& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }   
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }             
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::setParam(const std::string& key, const double& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                } 
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }               
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }  
}

bool YamlHelp::setParam(const std::string& key, const std::string& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }    
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }            
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }  
}

bool YamlHelp::setParam(const std::string& key, const std::vector<bool>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                } 
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }               
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::setParam(const std::string& key, const std::vector<int>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                } 
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }               
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::setParam(const std::string& key, const std::vector<double>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }  
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }              
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::setParam(const std::string& key, const std::vector<std::string>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }       
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }         
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }  
}

bool YamlHelp::addParam(const std::string& key, const bool& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }      
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }          
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }     
}

bool YamlHelp::addParam(const std::string& key, const int& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }     
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }           
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::addParam(const std::string& key, const double& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }   
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }             
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::addParam(const std::string& key, const std::string& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }   
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }             
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }    
}

bool YamlHelp::addParam(const std::string& key, const std::vector<bool>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }   
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }             
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::addParam(const std::string& key, const std::vector<int>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }       
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }         
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::addParam(const std::string& key, const std::vector<double>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }                
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::addParam(const std::string& key, const std::vector<std::string>& value)
{
    try
    {
        std::vector<std::string> key_set;
        getKeys(key, key_set);
        switch(key_set.size())
        {
            case 1:
                if(!node_[key_set[0]].IsDefined())
                {
                    node_[key_set[0]] = value;
                    return true;
                }
            case 2:
                if(!node_[key_set[0]][key_set[1]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]] = value;
                    return true;
                }       
            case 3: 
                if(node_[key_set[0]][key_set[1]][key_set[2]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]] = value;
                    return true;
                }
            case 4: 
                if(node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]].IsDefined())
                {
                    node_[key_set[0]][key_set[1]][key_set[2]][key_set[3]] = value;
                    return true;
                }         
        }
        return false;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }   
}

bool YamlHelp::deleteParam(const std::string& key)
{
    try
    {
        node_.remove(key);
        return true;
    }
    catch(YAML::Exception& exception)
    {
        std::cout<<exception.what()<<std::endl;
        return false;
    }
}

void YamlHelp::getKeys(const std::string& key_str, std::vector<std::string>& key_set)
{
    std::string key;
    size_t index_head = 0, index_tail = 0;
    do{
        index_tail = key_str.find('/', index_head);
        if(index_tail == std::string::npos)
        {
            key = key_str.substr(index_head);
            key_set.push_back(key);
            break;
        }
        else
        {
            key = key_str.substr(index_head, index_tail - index_head);
            key_set.push_back(key);
            index_head = index_tail + 1;
        }
    }while(1);
}

