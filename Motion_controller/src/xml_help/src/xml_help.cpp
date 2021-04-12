#include "xml_help.h"


using namespace base_space;

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


