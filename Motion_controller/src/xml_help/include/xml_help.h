#ifndef XML_HELP_H
#define XML_HELP_H

#include "common_log.h"
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <string>
#include <cstring>
#include <sstream>
#include <typeinfo>
#include <ros/ros.h>

namespace fst_base
{
class XmlHelp
{
public:
    XmlHelp();
    ~XmlHelp();

    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path);
    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlNodePtr node_ptr, const xmlChar* node_path);
    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path, const xmlChar* prefix, const xmlChar* url);
    xmlNodePtr* getNode(const xmlXPathObjectPtr object_ptr, unsigned int& node_num);
    bool getDataFromNode(const xmlDocPtr doc_ptr, const xmlNodePtr father_node_ptr, std::string& data, const xmlChar* child_node_path);

    template <typename T>
    bool getPropertyFromNode(const xmlNodePtr node_ptr, const xmlChar* property_name, T& data)
    {
        if(node_ptr == NULL)
        {
            return false;
        }
    
        xmlChar* node_data = xmlGetProp(node_ptr, property_name);
        if(node_data != NULL)
        {
            std::stringstream stream;
            stream<<(char*)node_data;
            stream>>data;
            xmlFree(node_data);
            return true;
        }
        else
        {
            return false;
        }
    }
    
    template <typename T>
    bool getPropertyFromNode(const xmlXPathObjectPtr xpath_ptr, unsigned int index, const xmlChar* property_name, T& data)
    {
        if(xpath_ptr == NULL)
        {
            return false;
        }
        xmlNodePtr node_ptr = xpath_ptr->nodesetval->nodeTab[index];
        return getPropertyFromNode<T>(node_ptr, property_name, data);
    }
    
    template <typename T>
    bool setPropertyToNode(const xmlNodePtr node_ptr, const xmlChar* property_name, T data)
    {
        if(node_ptr == NULL)
        {
            return false;
        }
    
        xmlAttrPtr attr_ptr;
        std::stringstream stream;
        stream<<data;
    
        attr_ptr = xmlSetProp(node_ptr, property_name, BAD_CAST(stream.str().c_str()));
        //attr_ptr = xmlSetProp(node_ptr, property_name, BAD_CAST(std::to_string(data).c_str()));
        
        return attr_ptr != NULL ? true:false;
    }
    
    template <typename T>
    bool setPropertyToNode(const xmlXPathObjectPtr xpath_ptr, unsigned int index, const xmlChar* property_name, T data)
    {
        if(xpath_ptr == NULL)
        {
            return false;
        }
        
        xmlNodePtr node_ptr = xpath_ptr->nodesetval->nodeTab[index];
        return setPropertyToNode<T>(node_ptr, property_name, data);
    }
    
    template <typename T>
    bool getDataFromNode(const xmlDocPtr doc_ptr, const xmlNodePtr father_node_ptr, T& data, const xmlChar* child_node_path)
    {
        if(doc_ptr == NULL
            || father_node_ptr == NULL)
        {
            return false;
        }
        unsigned int node_num;
        xmlNodePtr* child_node_ptr = getNode(getNodeObject(doc_ptr, father_node_ptr, child_node_path), node_num);
        if(node_num == 1 && child_node_ptr != NULL)
        {
            xmlChar* node_data = xmlNodeGetContent(child_node_ptr[0]);
            if(node_data != NULL)
            {
                std::stringstream stream;
                char* node_data_str = (char*)node_data;
                if(strlen(node_data_str) > 2
                    && node_data_str[0] == '#')
                {
                    node_data_str[0] = '0';
                    stream<<std::hex<<node_data_str;
                }
                else
                {   
                    stream << node_data_str;
                }
                stream >> data;
                xmlFree(node_data);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
private:
};

}


#endif


