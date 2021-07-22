#include "interpreter_register.h"
#include "reg_manager.h"
#include "log_manager_producer.h"

using namespace fst_ctrl;
using namespace log_space;

static RegManager reg_manager;

bool InterpReg_Init(void)
{
    return reg_manager.init();
}

ErrorCode InterpReg_GetRR(int id, RegValue *val)
{
    RValue value;
    reg_manager.getRRegValueById(id, value);
    val->rr = value.value;
    LogProducer::info("InterpReg", "get R[%d] register value %f", id, val->rr);
    return 0;
}

ErrorCode InterpReg_SetRR(int id, RegValue *val)
{
    RRegDataIpc data;
    data.id = id;
    data.value.value = val->rr;
    reg_manager.updateRRegValue(&data);
    LogProducer::info("InterpReg", "set R register[%d] value %f", id, val->rr);
    return 0;
}

ErrorCode InterpReg_GetSR(int id, RegValue *val)
{
    SrValue value;
    reg_manager.getSrRegValueById(id, value);
    memcpy(val->sr, value.value, STRING_REG_MAX_LENGTH);
    LogProducer::info("InterpReg", "get S register[%d] value %s", id, val->sr);
    return 0;
}

ErrorCode InterpReg_SetSR(int id, RegValue *val)
{
    SrRegDataIpc data;
    data.id = id;
    memcpy(data.value.value, val->sr, STRING_REG_MAX_LENGTH);
    reg_manager.updateSrRegValue(&data);
    LogProducer::info("InterpReg", "set S register[%d] value %s", id, val->sr);
    return 0;
}

ErrorCode InterpReg_GetMR(int id, RegValue *val)
{
    MrValue value;
    reg_manager.getMrRegValueById(id, value);
    val->mr = value.value;
    LogProducer::info("InterpReg", "get S register[%d] value %d", id, val->mr);
    return 0;
}

ErrorCode InterpReg_SetMR(int id, RegValue *val)
{
    MrRegDataIpc data;
    data.id = id;
    data.value.value = val->mr;
    reg_manager.updateMrRegValue(&data);
    LogProducer::info("InterpReg", "set M register[%d] value %d", id, val->mr);
    return 0;
}

ErrorCode InterpReg_GetPR(int id, RegValue *val)
{
    PrValue value;
    reg_manager.getPrRegValueById(id, value);
    LogProducer::info("InterpReg", "get P register[%d] coord:%d,group:%d," \
    "posture(%d,%d,%d,%d),turn(%d,%d,%d,%d%d,%d,%d,%d,%d),pos(%d,%d,%d,%d%d,%d,%d,%d,%d)", id, 
    val->pr.coord = value.pos_type,
    val->pr.group_id = value.group_id,
    val->pr.posture[0] = value.posture[0],
    val->pr.posture[1] = value.posture[1],
    val->pr.posture[2] = value.posture[2],
    val->pr.posture[3] = value.posture[3],
    val->pr.turn[0] = value.turn[0],
    val->pr.turn[1] = value.turn[1],
    val->pr.turn[2] = value.turn[2],
    val->pr.turn[3] = value.turn[3],
    val->pr.turn[4] = value.turn[4],
    val->pr.turn[5] = value.turn[5],
    val->pr.turn[6] = value.turn[6],
    val->pr.turn[7] = value.turn[7],
    val->pr.turn[8] = value.turn[8],
    val->pr.pos[0] = value.pos[0],
    val->pr.pos[1] = value.pos[1],
    val->pr.pos[2] = value.pos[2],
    val->pr.pos[3] = value.pos[3],
    val->pr.pos[4] = value.pos[4],
    val->pr.pos[5] = value.pos[5],
    val->pr.pos[6] = value.pos[6],
    val->pr.pos[7] = value.pos[7],
    val->pr.pos[8] = value.pos[8]);
    return 0;
}

ErrorCode InterpReg_SetPR(int id, RegValue *val)
{
    PrRegDataIpc data;

    LogProducer::info("InterpReg", "set P register[%d] coord:%d,group:%d," \
    "posture(%d,%d,%d,%d),turn(%d,%d,%d,%d%d,%d,%d,%d,%d),pos(%d,%d,%d,%d%d,%d,%d,%d,%d)", 
    data.id = id,
    data.value.pos_type = (uint8_t)val->pr.coord,
    data.value.group_id = val->pr.group_id,
    data.value.posture[0] = val->pr.posture[0],
    data.value.posture[1] = val->pr.posture[1],
    data.value.posture[2] = val->pr.posture[2],
    data.value.posture[3] = val->pr.posture[3],
    data.value.turn[0] = (int8_t)val->pr.turn[0],
    data.value.turn[1] = (int8_t)val->pr.turn[1],
    data.value.turn[2] = (int8_t)val->pr.turn[2],
    data.value.turn[3] = (int8_t)val->pr.turn[3],
    data.value.turn[4] = (int8_t)val->pr.turn[4],
    data.value.turn[5] = (int8_t)val->pr.turn[5],
    data.value.turn[6] = (int8_t)val->pr.turn[6],
    data.value.turn[7] = (int8_t)val->pr.turn[7],
    data.value.turn[8] = (int8_t)val->pr.turn[8],
    data.value.pos[0] = val->pr.pos[0],
    data.value.pos[1] = val->pr.pos[1],
    data.value.pos[2] = val->pr.pos[2],
    data.value.pos[3] = val->pr.pos[3],
    data.value.pos[4] = val->pr.pos[4],
    data.value.pos[5] = val->pr.pos[5],
    data.value.pos[6] = val->pr.pos[6],
    data.value.pos[7] = val->pr.pos[7],
    data.value.pos[8] = val->pr.pos[8]);

    reg_manager.updatePrRegPos(&data);

    return 0;
}
