
/*************************************************************************
	> File Name: parameter_manager_param_value.h
	> Author: 
	> Mail: 
	> Created Time: 2017年03月21日 星期二 17时22分54秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_PARAM_VALUE_H
#define _PARAMETER_MANAGER_PARAM_VALUE_H

#include <parameter_manager/parameter_manager_error_code.h>
#include <map>
#include <string>
#include <vector>
#include <map>
//#include <unordered_map>


#define FLOATING_PRECISION 0.0000000000001

namespace fst_parameter {

class ParamValue {

  public:
    enum ParamType {
        TYPEINVALID,
        TYPEBOOL,
        TYPEINT,
        TYPEDOUBLE,
        TYPESTRING,
        TYPEARRAY,
        TYPESTRUCT
    };
    
    typedef std::vector<ParamValue> ValueArray;
    //typedef std::unordered_map<std::string, ParamValue> ValueStruct;
    typedef std::map<std::string, ParamValue> ValueStruct;
    typedef ValueStruct::iterator iterator;
    
    //! Constructors
    ParamValue()             : type_(TYPEINVALID) { value_.asString = 0; }
    ParamValue(bool value)   : type_(TYPEBOOL)    { value_.asBool   = value; }
    ParamValue(int value)    : type_(TYPEINT)     { value_.asInt    = value; }
    ParamValue(double value) : type_(TYPEDOUBLE)  { value_.asDouble = value; }
    ParamValue(const char *value)        : type_(TYPESTRING) { value_.asString = new std::string(value); }
    ParamValue(const std::string &value) : type_(TYPESTRING) { value_.asString = new std::string(value); }

    //! Copy
    ParamValue(const ParamValue &rhs) : type_(TYPEINVALID) { *this = rhs; }

    //! Destructor
    ~ParamValue() { invalidParam(); }

    void clear() { invalidParam(); }

    // Operators
    ParamValue& operator=(const ParamValue &rhs);
    ParamValue& operator=(const bool &rhs)        { return operator=(ParamValue(rhs)); }
    ParamValue& operator=(const int &rhs)         { return operator=(ParamValue(rhs)); }
    ParamValue& operator=(const double &rhs)      { return operator=(ParamValue(rhs)); }
    ParamValue& operator=(const char *rhs)        { return operator=(ParamValue(rhs)); }
    ParamValue& operator=(const std::string &rhs) { return operator=(ParamValue(rhs)); }

    bool operator==(const ParamValue &other) const;
    bool operator!=(const ParamValue &other) const;

    operator bool&()        { assertTypeOrInvalid(TYPEBOOL);    return value_.asBool; }
    operator int&()         { assertTypeOrInvalid(TYPEINT);     return value_.asInt; }
    operator double()      { if(isTypeDoubleOrTypeInt()) return value_.asDouble; else return value_.asInt; }
    operator std::string&() { assertTypeOrInvalid(TYPESTRING);  return *value_.asString; }

    const ParamValue& operator[](int i) const   { assertArray(i+1); return value_.asArray->at(i); }
    ParamValue& operator[](int i)               { assertArray(i+1); return value_.asArray->at(i); }

    ParamValue& operator[](const std::string &k) { assertStruct(); return (*value_.asStruct)[k]; }
    ParamValue& operator[](const char *k) { assertStruct(); std::string s(k); return (*value_.asStruct)[s]; }
 
    iterator begin()    { assertStruct(); return (*value_.asStruct).begin(); }
    iterator end()      { assertStruct(); return (*value_.asStruct).end(); }

    // Accessors
    //! Return true if the value has been set to something.
    bool isValid()  const { return type_ != TYPEINVALID; }
    
    //! Return true if the value has been set to scalar, array or struct.
    bool isBool()   const { return type_ == TYPEBOOL; }
    bool isInt()    const { return type_ == TYPEINT; }
    bool isDouble() const { return type_ == TYPEDOUBLE; }
    bool isString() const { return type_ == TYPESTRING; }
    bool isScalar() const { return type_ == TYPEBOOL   || type_ == TYPEINT   || 
                                   type_ == TYPEDOUBLE || type_ == TYPESTRING; }
    bool isArray()  const { return type_ == TYPEARRAY; }
    bool isStruct() const { return type_ == TYPESTRUCT; }

    //! Return the type of the value stored. \see Type.
    const ParamType& getType() const { return type_; }

    //! Return the size for string, array, and struct values.
    int size() const;

    //! Specify the size for array values. Array values will grow beyond this size if needed.
    void setSize(int size) { assertArray(size); }

    //! Check for the existence of a struct member by name.
    bool hasMember(const std::string& name) const;
    //! Check for the existence of a struct member by name and delete it.
    bool delMember(const std::string& name);

    void setTag(const std::string &tag) { tag_ = tag; }
    void setTag(const char *tag)        { tag_ = tag; }

    const std::string& getTag() { return tag_; }

    //! Decode xml. Destroys any existing value.
    bool fromXml(const std::string &value_xml) { int offset = 0; return fromXml(value_xml, offset); }
    bool fromXml(const std::string &value_xml, int &offset);

    //! Encode the Value in xml
    std::string toXml() const;
    
    //! Write the value (no xml encoding)
    std::ostream& write(std::ostream& os) const;

  private:
    // Clean up
    void invalidParam();
    void assertTypeOrInvalid(ParamType t);
    bool isTypeDoubleOrTypeInt();       // true->TypeDouble  false->TypeInt
    void assertArray(int size) const;
    void assertArray(int size);
    void assertStruct();

    // XML decoding
    bool boolFromXml(const std::string &value_xml, int &offset);
    bool intFromXml(const std::string &value_xml, int &offset);
    bool doubleFromXml(const std::string &value_xml, int &offset);
    bool stringFromXml(const std::string &value_xml, int &offset);
    bool arrayFromXml(const std::string &value_xml, int &offset);
    bool structFromXml(const std::string &value_xml, int &offset);

    // XML encoding
    std::string boolToXml() const;
    std::string intToXml() const;
    std::string doubleToXml() const;
    std::string stringToXml() const;
    std::string arrayToXml() const;
    std::string structToXml() const;

    std::string tag_;
    ParamType type_;
    union {
        bool    asBool;
        int     asInt;
        double  asDouble;
        std::string*    asString;
        ValueArray*     asArray;
        ValueStruct*    asStruct;
    } value_;
};

//! A class representing an error.
//! If server methods throw this exception, a fault response is returned
//! to the client.
class ParamException {
  public:
    //! Constructor
    //!   @param message  A descriptive error message
    //!   @param code     An integer error code
    ParamException(const std::string &message, ErrorCode code = PARAM_INTERNAL_FAULT) :
        message_(message), code_(code) {}

    //! Return the error message.
    const std::string& getMessage() const { return message_; }

    //! Return the error code.
    const ErrorCode& getCode() const { return code_; }

  private:
    std::string message_;
    ErrorCode code_;
};

}  // namespace

// ostream
std::ostream& operator<<(std::ostream& os, fst_parameter::ParamValue &v);

#endif
