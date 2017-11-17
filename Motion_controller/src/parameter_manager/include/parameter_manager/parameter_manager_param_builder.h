#include <parameter_manager/parameter_manager_param_value.h>

#define INDENT_STEP     4
#define MAX_PRECISION   12        // 0.0000000000001
#define MINIMUM_DECIMAL FLOATING_PRECISION

namespace fst_parameter {

struct YamlLine {
	std::string  content;
	unsigned int indent;
    std::string  name;
	std::string  value;
};


class ParamBuilder {
	public:
		void buildParamFromString(const std::string &yaml_str, ParamValue &param_value);
		void dumpParamToString(ParamValue &param_value, std::string &yaml_str);

		private:
		void buildParamFromLines(std::vector<YamlLine>::iterator begin,
			                     std::vector<YamlLine>::iterator end,
			                     ParamValue &value);
		
        ParamValue getParamValueFromString(const std::string &str);
		ParamValue getStructFromString(std::string &str);
		ParamValue getArrayFromString(std::string &str);
		ParamValue getScalarFromString(std::string &str);

		std::string dumpStructToString(ParamValue &value, size_t indent);
		std::string dumpArrayToString(ParamValue &value, size_t indent);
		std::string dumpScalarToString(ParamValue &value, size_t indent);

		void cleanYamlString(std::string &str);
		void splitYamlString(const std::string &raw, std::vector<YamlLine> &cooked);
		void cleanYamlLines(std::vector<YamlLine> &lines);
		void analysisYamlLines(std::vector<YamlLine> &lines);

		unsigned int deleteWhiteSpaceInfrontOfString(std::string &str);
		unsigned int deleteWhiteSpaceAtEndOfString(std::string &str);
		unsigned int deleteZeroAtEndOfDouble(std::string &str);

		void deleteWhiteSpaceAtBothEnds(std::string &str);
		size_t deleteSpaceAround(std::string &str, size_t pos);
		
        bool isDummyLine(const YamlLine &line);
		bool isTypeInt(const std::string &str);
		bool isTypeHex(const std::string &str);
		
        int getHexFromString(const std::string &str);
	};


}
