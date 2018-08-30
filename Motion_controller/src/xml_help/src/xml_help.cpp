#include "xml_help.h"


using namespace fst_base;
using namespace std;


XmlHelp::XmlHelp()
{

}

XmlHelp::~XmlHelp()
{

}

xmlXPathObjectPtr XmlHelp::getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path)
{
	xmlXPathContextPtr context;
	xmlXPathObjectPtr result;
	context = xmlXPathNewContext(doc_ptr);
	result = xmlXPathEvalExpression(node_path, context);
	xmlXPathFreeContext(context);
	return result;
}

xmlXPathObjectPtr XmlHelp::getNodeObject(const xmlDocPtr doc_ptr, const xmlNodePtr node_ptr, const xmlChar* node_path)
{
	xmlXPathContextPtr context;
	xmlXPathObjectPtr result;
	context = xmlXPathNewContext(doc_ptr);
	result = xmlXPathNodeEval(node_ptr, node_path, context);
	xmlXPathFreeContext(context);
	return result;
}

xmlXPathObjectPtr XmlHelp::getNodeObject(const xmlDocPtr doc_ptr, const xmlChar* node_path, const xmlChar* prefix, const xmlChar* url)
{
	xmlXPathContextPtr context;
	xmlXPathObjectPtr result;
	context = xmlXPathNewContext(doc_ptr);
	xmlXPathRegisterNs(context, BAD_CAST prefix, BAD_CAST url);
	result = xmlXPathEvalExpression(node_path, context);
	xmlXPathFreeContext(context);
	return result;
}

xmlNodePtr* XmlHelp::getNode(const xmlXPathObjectPtr object_ptr, unsigned int& node_num)
{
	if(object_ptr == NULL)
	{
		return NULL;
	}
	node_num = (unsigned int)object_ptr->nodesetval->nodeNr;
	return object_ptr->nodesetval->nodeTab;
}

bool XmlHelp::getDataFromNode(const xmlDocPtr doc_ptr, const xmlNodePtr father_node_ptr, std::string& data, const xmlChar* child_node_path)
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
			data = (char*)node_data;
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


