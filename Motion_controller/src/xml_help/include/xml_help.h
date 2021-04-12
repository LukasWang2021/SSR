#ifndef XML_HELP_H
#define XML_HELP_H

/**
 * @file xml_help.h
 * @brief The file includes the class for handling file in XML format.
 * @author zhengyu.shen
 */

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <string>
#include <cstring>
#include <sstream>
#include <typeinfo>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief XmlHelp is the object to handle files in XML format.
 */
class XmlHelp
{
public:
    /**
     * @brief Constructor of the class.
     */     
    XmlHelp();
    /**
     * @brief Destructor of the class.
     */    
    ~XmlHelp();
    /**
     * @brief Get XPathObject by absolute node path.
     * @param [in] doc_ptr XML file pointer.
     * @param [in] node_path Node path.
     * @return The expected XPathObject.
     */
    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path);
    /**
     * @brief Get XPathObject by relative node path under specified node.
     * @param [in] doc_ptr XML file pointer.
     * @param [in] node_ptr Node pointer.
     * @param [in] node_path Node path.
     * @return The expected XPathObject.
     */    
    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlNodePtr node_ptr, const xmlChar* node_path);
    /**
     * @brief Get XPathObject by absolute node path in specified name space.
     * @param [in] doc_ptr XML file pointer.
     * @param [in] node_path Node path.
     * @param [in] prefix Prefix of the name space.
     * @param [in] url URL of the name space.
     * @return The expected XPathObject.
     */      
    xmlXPathObjectPtr getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path, const xmlChar* prefix, const xmlChar* url);
    /**
     * @brief Get xmlNode list in xmlXPathObject.
     * @param [in] object_ptr Pointer of xmlXPathObject.
     * @param [out] node_num The number of xmlNode.
     * @return Pointer of xmlNode list.
     */     
    xmlNodePtr* getNode(const xmlXPathObjectPtr object_ptr, unsigned int& node_num);
    /**
     * @brief Get property of a node.
     * @param [in] node_ptr Pointer of xmlNode.
     * @param [in] property_name Property name.
     * @param [out] data Property value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
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
    /**
     * @brief Get property of a node.
     * @param [in] xpath_ptr Pointer of xmlXPathObject.
     * @param [in] index The index of the xmlNode in the xmlXPathObject.
     * @param [in] property_name Property name.
     * @param [out] data Property value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
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
    /**
     * @brief Set property of a node.
     * @param [in] node_ptr Pointer of xmlNode.
     * @param [in] property_name Property name.
     * @param [in] data Property value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
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
    /**
     * @brief Set property of a node.
     * @param [in] xpath_ptr Pointer of xmlXPathObject.
     * @param [in] index The index of the xmlNode in the xmlXPathObject.
     * @param [in] property_name Property name.
     * @param [in] data Property value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */      
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
    /**
     * @brief Get value of a node.
     * @param [in] doc_ptr XML file pointer.
     * @param [in] father_node_ptr Pointer of the father xmlNode.
     * @param [in] child_node_path Node path relative to the father xmlNode.
     * @param [out] data Node value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
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
                    && node_data_str[0] == '0'
                    && node_data_str[1] == 'x')  // HEX data
                {
                    if(stream<<std::hex<<node_data_str
                        && stream>>data)
                    {
                        xmlFree(node_data);
                        return true;
                    }
                    else
                    {
                        xmlFree(node_data);
                        return false;
                    }
                }
                else    // DEC data or String
                {
                    if(stream << node_data_str
                        && stream >> data)
                    {
                        xmlFree(node_data);
                        return true;
                    }
                    else
                    {
                        xmlFree(node_data);
                        return false;
                    }
                }
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
    /**
     * @brief Get value of a node.
     * @param [in] node_ptr Pointer of xmlNode.
     * @param [out] data Node value.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    template <typename T>
    bool getDataFromNode(const xmlNodePtr node_ptr, T& data)
    {
        if(node_ptr == NULL)
        {
            return false;
        }
           
        xmlChar* node_data = xmlNodeGetContent(node_ptr);
        if(node_data != NULL)
        {
            std::stringstream stream;
            char* node_data_str = (char*)node_data; 
            if(strlen(node_data_str) > 2
                && node_data_str[0] == '0'
                && node_data_str[1] == 'x')  // HEX data
            {
                if(stream<<std::hex<<node_data_str
                    && stream>>data)
                {
                    xmlFree(node_data);
                    return true;
                }
                else
                {
                    xmlFree(node_data);
                    return false;
                }
            }
            else    // DEC data or String
            {
                if(stream << node_data_str
                    && stream >> data)
                {
                    xmlFree(node_data);
                    return true;
                }
                else
                {
                    xmlFree(node_data);
                    return false;
                }
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


