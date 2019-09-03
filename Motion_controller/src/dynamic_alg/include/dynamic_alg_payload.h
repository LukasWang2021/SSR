#ifndef DYNAMIC_ALG_PAYLOAD_H
#define DYNAMIC_ALG_PAYLOAD_H

#include "common_log.h"
#include <string>
#include <vector>
#include "parameter_manager/parameter_manager_param_group.h"

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
    std::string comment;
}PayloadInfo;

typedef struct
{
    int id;
    std::string comment;
}PayloadSummaryInfo;

class DynamicAlgPayload
{
public:
    DynamicAlgPayload(fst_log::Logger *logger);
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
    fst_log::Logger* log_ptr_;
    std::vector<PayloadInfo> payload_set_;
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
    int log_level_;
    int max_number_of_payloads_;

    void packDummyPayloadInfo(PayloadInfo& info);
    std::string getPayloadInfoPath(int coord_id);
    bool readAllPayloadInfoFromYaml(void);
    bool writePayloadInfoToYaml(PayloadInfo& info);
    void printfPayload(void);
};

}

#endif


