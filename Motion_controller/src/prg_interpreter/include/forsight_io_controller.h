#ifndef FORSIGHT_IO_CONTROLLER_H
#define FORSIGHT_IO_CONTROLLER_H
#include "forsight_inter_control.h"
#include "forsight_basint.h"

int forgesight_load_io_config();

eval_value forgesight_get_io_status(char *name);
int forgesight_set_io_status(char *name, eval_value& valueStart);

int forgesight_read_io_emulate_status(char * name, int& value);
int forgesight_mod_io_emulate_status(char * name, char value);
int forgesight_mod_io_emulate_value(char * name, char value);


#endif
