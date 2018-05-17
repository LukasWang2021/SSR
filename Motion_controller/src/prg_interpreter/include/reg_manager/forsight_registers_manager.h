#ifndef FORSIGHT_REGISTERS_MANAGER_H
#define FORSIGHT_REGISTERS_MANAGER_H
#include "forsight_inter_control.h"
#include "forsight_basint.h"

#define REGSITER_NAMES   "pr;sr;r;mr;uf;tf;pl"

int forgesight_registers_manager_get_register(
							struct thread_control_block* objThreadCntrolBlock, 
							char *name, eval_value * value);
int forgesight_registers_manager_set_register(
							struct thread_control_block* objThreadCntrolBlock, 
							char *name, eval_value * value);
// int forgesight_registers_manager_read_reg(RegMap & reg);
// int forgesight_registers_manager_mod_reg(RegMap & reg);
int forgesight_read_reg(RegMap & reg);
int forgesight_mod_reg(RegMap & reg);


#endif
