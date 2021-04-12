#ifndef DYNAMIC_ALG_PAYLOAD_H
#define DYNAMIC_ALG_PAYLOAD_H

#include <string>
#include <vector>
#include "common_error_code.h"
#include "yaml_help.h"

namespace basic_alg
{

typedef struct
{
    int id;
    bool is_valid;
    double m_load;
    double lcx_load;
    double lcy_load;
    double lcz_load;
    double Ixx_load;
    double Iyy_load;
    double Izz_load;
    char comment[256];
}PayloadInfo;

typedef struct
{
    int id;
    std::string comment;
}PayloadSummaryInfo;

class DynamicAlgPayload
{
public:
    DynamicAlgPayload();
    ~DynamicAlgPayload();

    bool init();

    ErrorCode addPayload(const PayloadInfo& info);
    ErrorCode deletePayload(int id);
    ErrorCode updatePayload(const PayloadInfo& info);
    ErrorCode movePayload(int expect_id, int original_id);
    ErrorCode getPayloadInfoById(int id, PayloadInfo& info);
    std::vector<PayloadSummaryInfo> getAllValidPayloadSummaryInfo(void);   
    void getAllValidPayloadSummaryInfo(std::vector<PayloadSummaryInfo>& info_list);

private:
    std::vector<PayloadInfo> payload_set_;
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
    int log_level_;
    int max_number_of_payloads_;

    void packDummyPayloadInfo(PayloadInfo& info);
    std::string getPayloadInfoPath(int coord_id);
    bool readAllPayloadInfoFromYaml(void);
    bool writePayloadInfoToYaml(const PayloadInfo& info);
    void printfPayload(void);
};

}

#endif


