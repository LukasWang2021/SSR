#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <cstring>
#include <sstream>

#include <parameter_manager/parameter_manager_param_builder.h>



using namespace std;

namespace fst_parameter {

	void ParamBuilder::buildParamFromString(const string &yaml_str, ParamValue &param_value)
	{
		string raw = yaml_str;
		vector<YamlLine> lines;
		vector<YamlLine>::iterator it;

		cleanYamlString(raw);
		splitYamlString(raw, lines);

		for (it = lines.begin(); it != lines.end(); )
		{
			deleteWhiteSpaceAtEndOfString(it->content);
			it->indent = deleteWhiteSpaceInfrontOfString(it->content);

			if (isDummyLine(*it))
			{
				it = lines.erase(it);
			}
			else
			{
				++it;
			}
		}

		analysisYamlLines(lines);
		buildParamFromLines(lines.begin(), lines.end(), param_value);

		/*
		catch (ParamException e) {
			std::cout << e.getMessage() << std::endl;
			std::cout << "Error while building ParamValue, error code=" << e.getCode() << std::endl;
			return false;
		}
		*/
	}

	void ParamBuilder::dumpParamToString(ParamValue &param_value, std::string &yaml_str)
	{
		yaml_str.clear();

		size_t indent = 0;

		if (param_value.isStruct())
		{
			yaml_str = dumpStructToString(param_value, indent);
		}
		else if (param_value.isArray())
		{
			yaml_str = dumpArrayToString(param_value, indent);
		}
		else if (param_value.isScalar())
		{
			yaml_str = dumpScalarToString(param_value, indent);
		}
		else
		{
			string error_string = "Dump error, type of param is invalid. Type = " + param_value.getType();
			throw ParamException(error_string, PARAM_TYPE_ERROR);
		}
	}

	string ParamBuilder::dumpStructToString(ParamValue &value, size_t indent)
	{
		string str;
		ParamValue::iterator it;

		for (it = value.begin(); it != value.end(); ++it)
		{
			if (it->second.isStruct()) {
				str = str + string(indent, ' ') + it->first + ":\n" + dumpStructToString(it->second, indent + INDENT_STEP);
			}
			else if (it->second.isArray()) {
				str = str + string(indent, ' ') + it->first + ":\n" + dumpArrayToString(it->second, indent + INDENT_STEP);
			}
			else if (it->second.isScalar()) {
				str = str + string(indent, ' ') + it->first + ": "  + dumpScalarToString(it->second, 0) + "\n";
			}
			else {
				string error_string = "Dump error, type of param is invalid. Type = " + it->second.getType();
				throw ParamException(error_string, PARAM_TYPE_ERROR);
			}
		}

		return str;
	}

	string ParamBuilder::dumpArrayToString(ParamValue &value, size_t indent)
	{
		string str;
		size_t size = value.size();

		for (size_t i = 0; i < size; ++i) {
			if (value[i].isStruct()) {
				// TODO
				string error_string = "Dump error, struct inside an array detected, which is not supportted by current version of ParamBuilder.";
				throw ParamException(error_string, PARAM_TYPE_ERROR);
			}
			else if (value[i].isArray()) {
				// TODO
				string error_string = "Dump error, array inside an array detected, which is not supportted by current version of ParamBuilder.";
				throw ParamException(error_string, PARAM_TYPE_ERROR);
			}
			else if (value[i].isScalar()) {
				str += string(indent, ' ') + "- " + dumpScalarToString(value[i], 0) + "\n";
			}
			else {
				string error_string = "Dump error, type of param is invalid. Type = " + value[i].getType();
				throw ParamException(error_string, PARAM_TYPE_ERROR);
			}
		}

		return str;
	}

	string ParamBuilder::dumpScalarToString(ParamValue &value, size_t indent)
	{
		string str;

		if (value.isDouble()) {
			char buf[32], form[8];
			double v = value;

			snprintf(form, sizeof(form), "%%0.%df", MAX_PRECISION);
			snprintf(buf, sizeof(buf), form, v);
			str = buf;

			if (str.length() > 0 && str[str.length() - 1] == '0')
				deleteZeroAtEndOfDouble(str);
		}
		else if (value.isInt()) {
			if (value.getTag() == "HEX") {
				int v = value;

				if (v < 0) {
					char buf[32];
					snprintf(buf, sizeof(buf), "%d", v);
					string error_string = "Dump HEX to string error, ParamValue(type = int) overflow, value=";
					error_string = error_string + buf;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
				}

				int tmp = 0;
				str = "0x";
				string::iterator pos = str.end();
				while (v > 0) {
					tmp = v % 16;
					v   = v / 16;

					if (tmp < 10)
						pos = str.insert(pos, '0' + tmp);
					else
						pos = str.insert(pos, 'A' + tmp - 10);
				}

				if (str == "0x") str = "0x0";
			}
			else {
				std::stringstream ss;
				ss << (int)value;
				ss >> str;
			}
		}
		else if (value.isString()) {
			str = (string)value;
		}
		else if (value.isBool()) {
			if ((bool)value)
				str = "true";
			else
				str = "false";
		}
		else {
			string error_string = "Dump error, type of param is not a scalar. Type = " + value.getType();
			throw ParamException(error_string, PARAM_TYPE_ERROR);
		}

		return str;
	}


	void ParamBuilder::buildParamFromLines(vector<YamlLine>::iterator begin,
		vector<YamlLine>::iterator end,
		ParamValue &value) {
		value.clear();

		// empty vector
		if (begin == end) {
			return;
		}
		// vector contains only one YAML line
		else if (begin + 1 == end) {
			if (!begin->name.empty() && !begin->value.empty()) {
				value[begin->name] = getParamValueFromString(begin->value);
			}
            else if (begin->name.empty() && !begin->value.empty()) {
                value[0] = getParamValueFromString(begin->value);
            }
			else
			{
				string error_string = "Cannot parse YAML string, name=" + begin->name + " value=" + begin->value;
				throw ParamException(error_string, PARAM_PARSE_ERROR);
			}
		}
		else {
			vector<YamlLine>::iterator it = begin;
			while (it != end) {
				if (!it->name.empty() && !it->value.empty()) {
					// there is a map element
					// std::cout << "single line" << std::endl;
					value[it->name] = getParamValueFromString(it->value);
					++it;
				}
				else if (!it->name.empty() && it->value.empty() && (it + 1 != end) && (it + 1)->indent > it->indent) {
					// there is a map
					// std::cout << "is a map" << std::endl;
					vector<YamlLine>::iterator temp_it = it + 1;
					while (temp_it != end && temp_it->indent > it->indent)  ++temp_it;
					ParamValue temp_value;
					buildParamFromLines(it + 1, temp_it, temp_value);
					value[it->name] = temp_value;
					it = temp_it;
				}
				else if (it->name.empty() && !it->value.empty()) {
					// there is a array
					// std::cout << "is a array" << std::endl;
					vector<YamlLine>::iterator temp_it = it;
					unsigned int cnt = 0;
					while (temp_it != end && temp_it->name.empty() &&
						!temp_it->value.empty() && temp_it->indent == it->indent)
					{
						size_t pos = temp_it->value.find_first_not_of(" ");
						value[cnt++] = getParamValueFromString(temp_it->value.substr(pos));
						++temp_it;
					}

					it = temp_it;
				}
				else {
					string error_string = "Cannot parse YAML string, name=" + it->name + " value=" + it->value;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
			}
		}
	}

	ParamValue ParamBuilder::getStructFromString(string &str)
	{
		ParamValue value;

		deleteWhiteSpaceAtBothEnds(str);
		if (str.size() > 0 && str[0] == '{' && str[str.length() - 1] == '}') {
			str = str.substr(1, str.size() - 2);
			deleteWhiteSpaceAtBothEnds(str);
		}
		else {
			string error_string = "Parse error, string=" + str;
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}
		
		size_t ptr = 0;
		size_t pos = str.find_first_of(':');
		if (pos == string::npos) {
			string error_string = "Parse error, string=" + str;
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}

		string struct_name = "";
		string struct_value = "";

		while (pos != string::npos) {
			pos = deleteSpaceAround(str, pos);
			ptr = 0;
			struct_name.clear();
			struct_value.clear();
			
			struct_name = str.substr(0, pos);
			if (struct_name.empty()) {
				string error_string = "Parse error, string=" + str;
				throw ParamException(error_string, PARAM_PARSE_ERROR);
			}

			if (str[pos + 1] == '{') {
				// struct_value is also a map
				size_t temp_ptr = pos + 1;
				size_t cnt = 0;

				while (temp_ptr < str.length()) {
					if (str[temp_ptr] == '{') cnt++;
					else if (str[temp_ptr] == '}') cnt--;
					if (cnt == 0) break;
					temp_ptr++;
				}

				if (cnt == 0) {
					struct_value = str.substr(pos + 1, temp_ptr - pos);
					value[struct_name] = getStructFromString(struct_value);
					str = str.substr(temp_ptr + 1);
				}
				else {
					// {  } mismatch detected
					string error_string = "Parse error, string=" + str;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
			}
			else if (str[pos + 1] == '[') {
				// struct_value is an array
				size_t temp_ptr = pos + 1;
				size_t cnt = 0;

				while (temp_ptr < str.length()) {
					if (str[temp_ptr] == '[') cnt++;
					else if (str[temp_ptr] == ']') cnt--;
					if (cnt == 0) break;
					temp_ptr++;
				}

				if (cnt == 0) {
					struct_value = str.substr(pos + 1, temp_ptr - pos);
					value[struct_name] = getArrayFromString(struct_value);
					str = str.substr(temp_ptr + 1);
				}
				else {
					// [  ] mismatch detected
					string error_string = "Parse error, string=" + str;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
			}
			else {
				// struct_value is an scalar
				size_t start_ptr = pos + 1;
				size_t end_ptr = str.find_first_of(",", start_ptr);

				if (end_ptr != string::npos) {
                    string s_str = str.substr(start_ptr, end_ptr - start_ptr);
					value[struct_name] = getScalarFromString(s_str);
					str = str.substr(end_ptr);
				}
				else {
                    string s_str = str.substr(start_ptr);
					value[struct_name] = getScalarFromString(s_str);
					str.clear();
				}
			}

			deleteWhiteSpaceInfrontOfString(str);
			if (!str.empty() && str[0] == ',') {
				str = str.substr(1);
				deleteWhiteSpaceInfrontOfString(str);
			}
			else if (!str.empty() && str[0] != ',') {
				string error_string = "Parse error, string=" + str;
				throw ParamException(error_string, PARAM_PARSE_ERROR);
			}

			pos = str.find_first_of(':');
		}

		return value;
	}

	ParamValue ParamBuilder::getArrayFromString(string &str)
	{
		ParamValue value;

		deleteWhiteSpaceAtBothEnds(str);
		if (str.length() > 0 && str[0] == '[' && str[str.length() - 1] == ']') {
			str = str.substr(1, str.length() - 2);
			deleteWhiteSpaceAtBothEnds(str);
		}
		else {
			string error_string = "Parse error, string=" + str;
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}

		string array_value = "";
		size_t array_cnt = 0;

		while (!str.empty()) {
			array_value.clear();

			if (str[0] == '{') {
				// array_value is a struct
				size_t temp_ptr = 0;
				size_t temp_cnt = 0;

				while (temp_ptr < str.length()) {
					if (str[temp_ptr] == '{') temp_cnt++;
					else if (str[temp_ptr] == '}') temp_cnt--;
					if (temp_cnt == 0) break;
					temp_ptr++;
				}

				if (temp_cnt == 0) {
					array_value = str.substr(0, temp_ptr + 1);
					value[array_cnt++] = getStructFromString(array_value);
					str = str.substr(temp_ptr + 1);
				}
				else {
					// {  } mismatch detected
					string error_string = "Parse error, string=" + str;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
			}
			else if (str[0] == '[') {
				// array_value is alse a array
				size_t temp_ptr = 0;
				size_t temp_cnt = 0;

				while (temp_ptr < str.length()) {
					if (str[temp_ptr] == '[') temp_cnt++;
					else if (str[temp_ptr] == ']') temp_cnt--;
					if (temp_cnt == 0) break;
					temp_ptr++;
				}

				if (temp_cnt == 0) {
					array_value = str.substr(0, temp_ptr + 1);
					value[array_cnt++] = getArrayFromString(array_value);
					str = str.substr(temp_ptr + 1);
				}
				else {
					// [  ] mismatch detected
					string error_string = "Parse error, string=" + str;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
			}
			else {
				// struct_value is an scalar
				size_t start_ptr = 0;
				size_t end_ptr = str.find_first_of(",", start_ptr);

				if (end_ptr != string::npos) {
                    string s_str = str.substr(start_ptr, end_ptr - start_ptr);
					value[array_cnt++] = getScalarFromString(s_str);
					str = str.substr(end_ptr);
				}
				else {
                    string s_str = str.substr(start_ptr);
					value[array_cnt++] = getScalarFromString(s_str);
					str.clear();
				}
			}

			deleteWhiteSpaceInfrontOfString(str);

			if (!str.empty() && str[0] == ',') {
				str = str.substr(1);
				deleteWhiteSpaceInfrontOfString(str);
			}
			else if (!str.empty() && str[0] != ',') {
				string error_string = "Parse error, string=" + str;
				throw ParamException(error_string, PARAM_PARSE_ERROR);
			}
		}

		return value;
	}

	ParamValue ParamBuilder::getScalarFromString(string &str)
	{
		ParamValue value;

		deleteWhiteSpaceAtBothEnds(str);

		if (str.empty()) {
			string error_string = "Parse error, scalar string is empty.";
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}
		
		if (str.size() > 1 && str[0] == '\'' && str[str.length() - 1] == '\'' ||
			str.size() > 1 && str[0] == '\"' && str[str.length() - 1] == '\"')
		{
			// Is a string
			value = str.substr(1, str.size() - 2);
			return value;
		}
		else if (str.size() == 1 && (str[0] == '\'' || str[0] == '\"')) {
			string error_string = "Parse error, string=" + str;
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}
		else {
			// Is a scalar

			if (isTypeInt(str)) {
				// type int
				std::stringstream ss(str);
				int int_value;
				ss >> int_value;
				value = int_value;
				return value;
			}
			
			if (str.size() > 2 && str[0] == '0' && str[1] == 'x') {
				str.erase(0, 2);
				if (isTypeHex(str)) {
					// type hex
					if (str.size() < 8 || (str.size() == 8 && str[0] >= '0' && str[0] <= '9')) {
						value = getHexFromString(str);
						value.setTag("HEX");
						return value;
					}
					else {
						string error_string = "Parse HEX-string to ParamValue, ParamValue(type = int) overflow, value=0x" + str;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
				}
				else {
					str = "0x" + str;
				}
			}

			// type double
			std::stringstream ss(str);
			double double_value;
			ss >> double_value;
			if (ss.eof()) {
				value = double_value;
				return value;
			}

			// type bool
			if (str == "true" || str == "True" || str == "TRUE") {
				value = true;
				return value;
			}
			else if (str == "false" || str == "False" || str == "FALSE") {
				value = false;
				return value;
			}

			// type string
			value = str;
			return value;
		}

		string error_string = "ParamBuilder internal fault, string=" + str;
		throw ParamException(error_string, PARAM_INTERNAL_FAULT);
	}

	int ParamBuilder::getHexFromString(const std::string &str)
	{
		size_t  pos = 0;
		int     res = 0;

		while (pos < str.length()) {
			res = res * 16;

			if (str[pos] >= '0' && str[pos] <= '9')
				res += str[pos] - '0';
			else if (str[pos] >= 'A' && str[pos] <= 'F')
				res += str[pos] - 'A' + 10;
			else if (str[pos] >= 'a' && str[pos] <= 'f')
				res += str[pos] - 'a' + 10;

			pos++;
		}

		return res;
	}

	ParamValue ParamBuilder::getParamValueFromString(const string &str)
	{
		string copy_str(str);
		deleteWhiteSpaceAtBothEnds(copy_str);
		
		if (copy_str.empty()) {
			string error_string = "Parse error, try to build a ParamValue from empty string";
			throw ParamException(error_string, PARAM_PARSE_ERROR);
		}

		if (copy_str[0] == '{' && copy_str[copy_str.length() - 1] == '}') {
			// Is a map
			// copy_str = copy_str.substr(1, copy_str.size() - 2);
			// deleteWhiteSpaceAtBothEnds(copy_str);

			return getStructFromString(copy_str);
		}
		else if (copy_str[0] == '[' && copy_str[copy_str.length() - 1] == ']') {
			// Is an array
			// copy_str = copy_str.substr(1, copy_str.size() - 2);
			// deleteWhiteSpaceAtBothEnds(copy_str);

			return getArrayFromString(copy_str);
		}
		else {
			// Is a scalar
			return getScalarFromString(copy_str);
		}
	}

	void ParamBuilder::analysisYamlLines(vector<YamlLine> &lines) {
		vector<YamlLine>::iterator it;
		//std::cout << "YAML Lines:" << std::endl;
		//for (it = lines.begin(); it != lines.end(); ++it)
		//   std::cout << it->content << std::endl;
		for (it = lines.begin(); it != lines.end(); ++it) {
			size_t notes = it->content.find_first_of('#');

			if (notes != string::npos) {
				it->content.erase(notes);
				deleteWhiteSpaceAtEndOfString(it->content);
			}

			if (it->content[0] == '-') {
				//it->name.clear();
				deleteSpaceAround(it->content, 0);
				it->value = it->content.substr(1);
			}
			else {
				size_t pos = it->content.find_first_of(':');

				if (pos != string::npos) {
					if (pos != 0) {
						pos = deleteSpaceAround(it->content, pos);
						it->name = it->content.substr(0, pos);
						it->value = it->content.substr(pos + 1);
					}
					else {
						string error_string = "Format error, string=" + it->content;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
				}
				else {
					string error_string = "Format error, string=" + it->content;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
					//it->name.clear();
					//if (pos == string::npos)    it->value = it->content;
					//else                        it->value = it->content.substr(pos + 1);
				}
			}
		}

		for (it = lines.begin(); it != lines.end(); ++it) {
			if (it->content[0] == '-') {
				if (it == lines.begin()) {
					string error_string = "A valid config file should begin as a struct, string=" + it->content;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
				else {
					if ((it - 1)->content[0] == '-' && (it - 1)->indent != it->indent) {
						string error_string = "Format error:\n" + (it - 1)->content + "\n" + it->content;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
					if ((it - 1)->content[0] != '-' && (it - 1)->indent >= it->indent) {
						string error_string = "Format error:\n" + (it - 1)->content + "\n" + it->content;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
				}
				if ((it + 1) != lines.end()) {
					if ((it + 1)->content[0] == '-' && (it + 1)->indent != it->indent) {
						string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
					if ((it + 1)->content[0] != '-' && (it + 1)->indent >= it->indent) {
						string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
						throw ParamException(error_string, PARAM_PARSE_ERROR);
					}
				}
			}
			else {
				if (it->name.empty()) {
					string error_string = "A struct line should begin with a name, string=" + it->content;
					throw ParamException(error_string, PARAM_PARSE_ERROR);
				}
				else {
					if (!it->value.empty()) {
						if ((it + 1) != lines.end() && (it + 1)->indent > it->indent) {
							string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
							throw ParamException(error_string, PARAM_PARSE_ERROR);
						}
					}
					else {
						if ((it + 1) == lines.end() || (it + 1)->indent <= it->indent) {
							string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
							throw ParamException(error_string, PARAM_PARSE_ERROR);
						}
					}
				}
			}
		}
	}

	void ParamBuilder::cleanYamlLines(vector<YamlLine> &lines) {
		vector<YamlLine>::iterator it;

		for (it = lines.begin(); it != lines.end(); ) {

			deleteWhiteSpaceAtEndOfString(it->content);
			
			if (isDummyLine(*it)) {
				it = lines.erase(it);
			}
			else {
				++it;
			}
		}
	}

	void ParamBuilder::deleteWhiteSpaceAtBothEnds(string &str)
	{
		deleteWhiteSpaceAtEndOfString(str);
		deleteWhiteSpaceInfrontOfString(str);
	}

	// Delete spaces around the character specified by pos in str
	// If pos >= the length of str, then delete all the spaces at the end of str
	size_t ParamBuilder::deleteSpaceAround(string &str, size_t pos)
	{
		if (pos >= str.length()) pos = str.length();

		size_t ptr = pos + 1;
		size_t cnt = 0;

		while (ptr < str.length() && str[ptr] == ' ') {
			cnt++;
			ptr++;
		}

		if (cnt > 0)  str.erase(pos + 1, cnt);
		if (pos == 0) return pos;

		ptr = pos - 1;
		cnt = 0;

		while (ptr < str.length() && str[ptr] == ' ') {
			cnt++;
			ptr--;
		}

		if (cnt > 0)  str.erase(pos - cnt, cnt);
		return pos - cnt;
	}

	bool ParamBuilder::isDummyLine(const YamlLine &line)
	{
		size_t pos = line.content.find_first_not_of(" \r\n\t\v\f");

		if (pos == string::npos)        return true;
		if (line.content[pos] == '#')   return true;
		
		return false;
	}

	void ParamBuilder::splitYamlString(const string &raw, vector<YamlLine> &cooked)
	{
		cooked.clear();
		if (raw.empty()) return;

		YamlLine yaml_line;
		yaml_line.indent = 0;
		string sep = "\n";
		string temp;
		string::size_type begin = raw.find_first_not_of(sep);
		string::size_type position = 0;

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
				yaml_line.content = temp;
				cooked.push_back(yaml_line);
				temp.clear();
			}
		}
	}

	void ParamBuilder::cleanYamlString(string &str) {
		if (str.empty()) return;

		//cout << "raw_str length=" << str.length() << ":" << endl << str << endl;

		deleteWhiteSpaceAtEndOfString(str);
		deleteWhiteSpaceInfrontOfString(str);

		// delete all '\r' '\t' '\v' '\f' in the string
		size_t pos = str.find_first_of("\r\t\v\f");

		while (pos != string::npos) {
			str.erase(pos, 1);
			pos = str.find_first_of("\r\t\v\f", pos);
		}

		//cout << "cooked str length=" << str.length() << ":" << endl << str << endl;
	}

	unsigned int ParamBuilder::deleteWhiteSpaceInfrontOfString(string &str)
	{
		// delete white spaces in front of the string
		size_t pos = str.find_first_not_of(" \r\n\t\v\f");

		if (pos != string::npos) {
			str.erase(0, pos);
			return pos;
		}
		else {
			size_t len = str.length();
			str.clear();
			return len;
		}
	}

	unsigned int ParamBuilder::deleteWhiteSpaceAtEndOfString(string &str)
	{
		// delete white spaces at the end of string
		size_t pos = str.find_last_not_of(" \r\n\t\v\f");

		if (pos != string::npos) {
			if (pos != str.length() - 1) {
				size_t org_len = str.length();
				str.erase(pos + 1);
				return org_len - str.length();
			}
			else {
				return 0;
			}
		}
		else {
			size_t org_len = str.length();
			str.clear();
			return org_len;
		}
	}

	unsigned int ParamBuilder::deleteZeroAtEndOfDouble(string &str)
	{
		// delete zeros at the end of double string
		size_t tmp = str.find_last_of('.');

		if (tmp != string::npos) {
			size_t pos = str.find_last_not_of('0');
			size_t org_len = str.length();
			
			if (pos == tmp) {
				str.erase(tmp + 2);
			}
			else {
				str.erase(pos + 1);
			}

			return org_len - str.length();
		}
		else {
			str.clear();
			return 0;
		}
	}

	bool ParamBuilder::isTypeInt(const string &str)
	{
		if (!str.empty()) {
			size_t pos = 0;

			if (str[0] == '+' || str[0] == '-') {
				if (str.length() == 1)
					return false;
				pos = 1;
			}

			while (pos < str.length()) {
				if (str[pos] > '9' || str[pos] < '0')
					return false;
				pos++;
			}

			return true;
		}
		else {
			return false;
		}
	}

	bool ParamBuilder::isTypeHex(const string &str)
	{
		if (!str.empty())
		{
			size_t pos = 0;

			while (pos < str.length()) {
				if ((str[pos] < '0' || str[pos] > '9') && (str[pos] < 'A' || str[pos] > 'F') && (str[pos] < 'a' || str[pos] > 'f'))
					return false;
				pos++;
			}

			return true;
		}
		else {
			return false;
		}
	}
}
