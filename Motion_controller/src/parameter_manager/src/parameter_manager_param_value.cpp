#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <ostream>
#include <cmath>
#include <ctype.h>
#include <string.h>
#include <parameter_manager/parameter_manager_param_value.h>



#define MINIMUM_DOUBLE_POINT 0.0000000001

using std::string;

namespace fst_parameter {

static const char VALUE_TAG[]   = "<v>";
static const char VALUE_ETAG[]  = "</v>";

static const char BOOL_TAG[]    = "<b>";
static const char BOOL_ETAG[]   = "</b>";
static const char INT_TAG[]     = "<i>";
static const char INT_ETAG[]    = "</i>";
static const char DOUBLE_TAG[]  = "<d>";
static const char DOUBLE_ETAG[] = "</d>";

static const char ARRAY_TAG[]   = "<a>";
static const char DATA_TAG[]    = "<e>";
static const char DATA_ETAG[]   = "</e>";
static const char ARRAY_ETAG[]  = "</a>";

static const char STRUCT_TAG[]  = "<s>";
static const char MEMBER_TAG[]  = "<m>";
static const char NAME_TAG[]    = "<n>";
static const char NAME_ETAG[]   = "</n>";
static const char MEMBER_ETAG[] = "</m>";
static const char STRUCT_ETAG[] = "</s>";

string getNextTag(const string &xml, int &offset);
bool nextTagIs(const char* tag, const string &xml, int &offset);
bool findTag(const char *tag, const string &xml, int &offset);
string parseTag(const char *tag, const string &xml, int &offset);

int ParamValue::size() const {
    switch (type_) {
        case TYPESTRING: return int(value_.asString->size());
        case TYPEARRAY:  return int(value_.asArray->size());
        case TYPESTRUCT: return int(value_.asStruct->size());
        default: break;
    }

    throw ParamException("Type error", PARAM_TYPE_ERROR);
}

bool ParamValue::hasMember(const string &name) const {
    return type_ == TYPESTRUCT && value_.asStruct->find(name) != value_.asStruct->end();
}

bool ParamValue::delMember(const string& name) {
    if (hasMember(name)) {
        value_.asStruct->erase(name);
        return true;
    }
    else {
        return false;
    }
}

// Operators
ParamValue& ParamValue::operator=(const ParamValue &rhs) {
    if (this != &rhs) {
        invalidParam();
        type_ = rhs.type_;
        tag_  = rhs.tag_;
        switch (type_) {
            case TYPEBOOL:      value_.asBool   = rhs.value_.asBool;    break;
            case TYPEINT:       value_.asInt    = rhs.value_.asInt;     break;
            case TYPEDOUBLE:    value_.asDouble = rhs.value_.asDouble;  break;
            case TYPESTRING:    value_.asString = new string(*rhs.value_.asString);    break;
            case TYPEARRAY:     value_.asArray  = new ValueArray(*rhs.value_.asArray);      break;
            case TYPESTRUCT:    value_.asStruct = new ValueStruct(*rhs.value_.asStruct);    break;
            default:            value_.asString = 0;    break;
        }
    }
    return *this;
}

bool ParamValue::operator==(const ParamValue &other) const {
    if (type_ != other.type_)
        return false;

    switch (type_) {
        case TYPEBOOL:      return !value_.asBool && !other.value_.asBool || value_.asBool && other.value_.asBool;
        case TYPEINT:       return value_.asInt == other.value_.asInt;
        case TYPEDOUBLE:    return fabs(value_.asDouble - other.value_.asDouble) < MINIMUM_DOUBLE_POINT;
        case TYPESTRING:    return *value_.asString == *other.value_.asString;
        case TYPEARRAY:     return *value_.asArray == *other.value_.asArray;

        // The map<>::operator== requires the definition of value< for kcc
        case TYPESTRUCT:   //return *_value.asStruct == *other._value.asStruct;
        {
            if (value_.asStruct->size() != other.value_.asStruct->size())
                return false;
            ValueStruct::const_iterator it1 = value_.asStruct->begin();
            ValueStruct::const_iterator it2 = other.value_.asStruct->begin();
            while (it1 != value_.asStruct->end()) {
                const ParamValue& v1 = it1->second;
                const ParamValue& v2 = it2->second;
                if ( ! (v1 == v2))
                    return false;
                ++it1;
                ++it2;
            }
            return true;
        }
        default: break;
    }
    return true;    // Both invalid values ...
}

bool ParamValue::operator!=(const ParamValue &other) const {
    return !(*this == other);
}

void ParamValue::invalidParam() {
    switch (type_) {
        case TYPESTRING:    delete value_.asString; break;
        case TYPEARRAY:     delete value_.asArray;  break;
        case TYPESTRUCT:    delete value_.asStruct; break;
        default: break;
    }
    tag_.clear();
    type_ = TYPEINVALID;
    value_.asString = 0;
}


void ParamValue::assertTypeOrInvalid(ParamType t) {
    if (type_ == TYPEINVALID) {
        type_ = t;
        switch (type_) {    // Ensure there is a valid value for the type
            case TYPESTRING:   value_.asString = new string(); break;
            case TYPEARRAY:    value_.asArray  = new ValueArray();  break;
            case TYPESTRUCT:   value_.asStruct = new ValueStruct(); break;
            default:           value_.asString = 0; break;
        }
    }
    else if (type_ != t) {
        throw ParamException("Type error", PARAM_TYPE_ERROR);
    }
}

void ParamValue::assertArray(int size) const {
    if (type_ != TYPEARRAY)
        throw ParamException("Type error: expected an array", PARAM_TYPE_ERROR);
    else if (int(value_.asArray->size()) < size)
        throw ParamException("Range error: array index too large", PARAM_LENGTH_ERROR);
}

void ParamValue::assertArray(int size) {
    if (type_ == TYPEINVALID) {
        type_ = TYPEARRAY;
        value_.asArray = new ValueArray(size);
    }
    else if (type_ == TYPEARRAY) {
        if (int(value_.asArray->size()) < size)
        value_.asArray->resize(size);
    }
    else {
        throw ParamException("Type error: expected an array", PARAM_TYPE_ERROR);
    }
}

void ParamValue::assertStruct() {
    if (type_ == TYPEINVALID) {
        type_ = TYPESTRUCT;
        value_.asStruct = new ValueStruct();
    }
    else if (type_ != TYPESTRUCT) {
        throw ParamException("Type error: expected a struct", PARAM_TYPE_ERROR);
    }
}

// Set the value from xml. The chars at *offset into valueXml 
// should be the start of a <value> tag. Destroys any existing value.
bool ParamValue::fromXml(const string &valueXml, int &offset) {
    int saved_offset = offset;

    invalidParam();
    if (!nextTagIs(VALUE_TAG, valueXml, offset))
      return false;       // Not a value, offset not updated

	int after_value_offset = offset;
    string type_tag = getNextTag(valueXml, offset);
    bool result = false;
    if (type_tag == BOOL_TAG)
        result = boolFromXml(valueXml, offset);
    else if (type_tag == INT_TAG)
        result = intFromXml(valueXml, offset);
    else if (type_tag == DOUBLE_TAG)
        result = doubleFromXml(valueXml, offset);
    else if (type_tag.empty())
        result = stringFromXml(valueXml, offset);
    else if (type_tag == ARRAY_TAG)
        result = arrayFromXml(valueXml, offset);
    else if (type_tag == STRUCT_TAG)
        result = structFromXml(valueXml, offset);
    // Watch for empty/blank strings with no <string>tag
    else if (type_tag == VALUE_ETAG) {
      offset = after_value_offset;   // back up & try again
      result = stringFromXml(valueXml, offset);
    }

    if (result)  // Skip over the </value> tag
        findTag(VALUE_ETAG, valueXml, offset);
    else        // Unrecognized tag after <value>
      offset = saved_offset;

    return result;
}

// Encode the Value in xml
string ParamValue::toXml() const {
    switch (type_) {
        case TYPEBOOL:    return boolToXml();
        case TYPEINT:     return intToXml();
        case TYPEDOUBLE:  return doubleToXml();
        case TYPESTRING:  return stringToXml();
        case TYPEARRAY:   return arrayToXml();
        case TYPESTRUCT:  return structToXml();
        default: break;
    }
    return string();   // Invalid value
}

// Bool
bool ParamValue::boolFromXml(const string &value_xml, int &offset) {
    const char *value_start = value_xml.c_str() + offset;
    char *value_end;
    long ivalue = strtol(value_start, &value_end, 10);
    if (value_end == value_start || (ivalue != 0 && ivalue != 1))
        return false;

    type_ = TYPEBOOL;
    value_.asBool = (ivalue == 1);
    offset += int(value_end - value_start);
    return true;
}

string ParamValue::boolToXml() const {
    string xml = VALUE_TAG;
    xml += BOOL_TAG;
    xml += (value_.asBool ? "1" : "0");
    xml += BOOL_ETAG;
    xml += VALUE_ETAG;
    return xml;
}

// Int
bool ParamValue::intFromXml(const string &value_xml, int &offset) {
    const char *value_start = value_xml.c_str() + offset;
    char *value_end;
    long ivalue = strtol(value_start, &value_end, 10);
    if (value_end == value_start)
        return false;

    type_ = TYPEINT;
    value_.asInt = int(ivalue);
    offset += int(value_end - value_start);
    return true;
}

string ParamValue::intToXml() const {
    /*
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", value_.asInt);
    string xml = VALUE_TAG;
    xml += INT_TAG;
    xml += buf;
    xml += INT_ETAG;
    xml += VALUE_ETAG;
    */
    std::stringstream ss;
    ss << value_.asInt;

    string xml = VALUE_TAG;
    xml += INT_TAG;
    xml += ss.str();
    xml += INT_ETAG;
    xml += VALUE_ETAG;
    return xml;
}

// Double
bool ParamValue::doubleFromXml(const string &value_xml, int &offset) {
    const char *value_start = value_xml.c_str() + offset;
    char *value_end;

    double dvalue = strtod(value_start, &value_end);
    if (value_end == value_start)
        return false;

    type_ = TYPEDOUBLE;
    value_.asDouble = dvalue;
    offset += int(value_end - value_start);
    return true;
}

string ParamValue::doubleToXml() const {
    // ticket #2438
    std::stringstream ss;
    ss.precision(16);
    ss << value_.asDouble;

    string xml = VALUE_TAG;
    xml += DOUBLE_TAG;
    xml += ss.str();
    xml += DOUBLE_ETAG;
    xml += VALUE_ETAG;
    return xml;
}

// String
bool ParamValue::stringFromXml(const string &value_xml, int &offset) {
    size_t value_end = value_xml.find('<', offset);
    if (value_end == string::npos)
        return false;     // No end tag;

    type_ = TYPESTRING;
    value_.asString = new string(value_xml.substr(offset, value_end - offset));
    offset += int(value_.asString->length());
    return true;
}

string ParamValue::stringToXml() const {
    string xml = VALUE_TAG;
    xml += *value_.asString;
    xml += VALUE_ETAG;
    return xml;
}

// Array
bool ParamValue::arrayFromXml(const string &value_xml, int &offset) {
    if (!nextTagIs(DATA_TAG, value_xml, offset))
        return false;

    type_ = TYPEARRAY;
    value_.asArray = new ValueArray;
    ParamValue v;
    while (v.fromXml(value_xml, offset))
        value_.asArray->push_back(v);       // copy...

    // Skip the trailing </data>
    nextTagIs(DATA_ETAG, value_xml, offset);
    return true;
}

string ParamValue::arrayToXml() const {
    string xml = VALUE_TAG;
    xml += ARRAY_TAG;
    xml += DATA_TAG;

    int s = value_.asArray->size();
    for (int i = 0; i < s; ++i)
       xml += value_.asArray->at(i).toXml();

    xml += DATA_ETAG;
    xml += ARRAY_ETAG;
    xml += VALUE_ETAG;
    return xml;
}

// Struct
bool ParamValue::structFromXml(const string &value_xml, int &offset) {
    type_ = TYPESTRUCT;
    value_.asStruct = new ValueStruct;

    while (nextTagIs(MEMBER_TAG, value_xml, offset)) {
        // name
        const string name = parseTag(NAME_TAG, value_xml, offset);
        // value
        ParamValue v;
        v.fromXml(value_xml, offset);
        if (!v.isValid()) {
            invalidParam();
            return false;
        }
        const std::pair<const string, ParamValue> p(name, v);
        value_.asStruct->insert(p);

        nextTagIs(MEMBER_ETAG, value_xml, offset);
    }
    return true;
}

string ParamValue::structToXml() const {
    string xml = VALUE_TAG;
    xml += STRUCT_TAG;

    ValueStruct::const_iterator it;
    for ( it = value_.asStruct->begin(); it != value_.asStruct->end(); ++it) {
        xml += MEMBER_TAG;
        xml += NAME_TAG;
        xml += it->first;
        xml += NAME_ETAG;
        xml += it->second.toXml();
        xml += MEMBER_ETAG;
    }

    xml += STRUCT_ETAG;
    xml += VALUE_ETAG;
    return xml;
}

// Write the value without xml encoding it
std::ostream& ParamValue::write(std::ostream &os) const {
    switch (type_) {
        case TYPEBOOL:      os << (value_.asBool ? "true" : "false");    break;
        case TYPEINT:       os << value_.asInt;     break;
        case TYPEDOUBLE:    os << value_.asDouble;  break;
        case TYPESTRING:    os << *value_.asString; break;
        case TYPEARRAY:
        {
            int s = int(value_.asArray->size());
            os << '[';
            for (int i = 0; i < s; ++i) {
                if (i > 0) 
                    os << ',';
                value_.asArray->at(i).write(os);
            }
            os << ']';
            break;
        }
        case TYPESTRUCT:
        {
            os << '{';
            ValueStruct::const_iterator it;
            for (it = value_.asStruct->begin(); it != value_.asStruct->end(); ++it) {
                if (it != value_.asStruct->begin())
                    os << ',';
                os << it->first << ':';
                it->second.write(os);
            }
            os << '}';
            break;
        }
        default: break;
    }

    return os;
}

// Returns true if the tag is found at the specified offset (modulo any whitespace)
// and updates offset to the char after the tag
bool nextTagIs(const char* tag, const string &xml, int &offset) {
    if (offset >= int(xml.length()))
        return false;
    
    const char *cp = xml.c_str() + offset;
    int nc = 0;
    while (*cp && isspace(*cp)) {
        ++cp;
        ++nc;
    }

    int len = int(strlen(tag));
    if (*cp && (strncmp(cp, tag, len) == 0)) {
        offset += nc + len;
        return true;
    }

    return false;
}

// Returns the next tag and updates offset to the char after the tag, or empty string
// if the next non-whitespace character is not '<'
string getNextTag(const string &xml, int &offset) {
    if (offset >= int(xml.length()))
        return std::string();

    int pos = offset;
    const char *cp = xml.c_str() + pos;
    while (*cp && isspace(*cp)) {
        ++cp;
        ++pos;
    }

    if (*cp != '<')
        return string();

    string s;
    do {
        s += *cp;
        ++pos;
    } while (*cp++ != '>' && *cp != 0);

    offset = pos;
    return s;
}

// Returns true if the tag is found and updates offset to the char after the tag
bool findTag(const char *tag, const string &xml, int &offset) {
    if (offset >= int(xml.length()))
        return false;
    
    size_t istart = xml.find(tag, offset);
    if (istart == string::npos)
        return false;

    offset = int(istart + strlen(tag));
    return true;
}

// Returns contents between <tag> and </tag>, updates offset to char after </tag>
string parseTag(const char *tag, const string &xml, int &offset) {
    if (offset >= int(xml.length()))
        return string();
    
    size_t istart = xml.find(tag, offset);
    if (istart == string::npos)
        return string();
    istart += strlen(tag);
    string etag = "</";
    etag += tag + 1;
    size_t iend = xml.find(etag, istart);
    if (iend == string::npos)
        return string();

    offset = int(iend + etag.length());
    return xml.substr(istart, iend - istart);
}

}  // namespace fst_parameter

// ostream
std::ostream& operator<<(std::ostream& os, fst_parameter::ParamValue &v) { 
  return v.write(os);
}


