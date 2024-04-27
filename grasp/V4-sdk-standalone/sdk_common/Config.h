#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <map>
#include <string>
#include <stdint.h>

typedef enum
{
	PARAM_BOOLEAN = 0,
	PARAM_INT32 = 1,
	PARAM_FLOAT = 2,
	PARAM_ENUM = 3,
} param_type_t;

typedef enum
{
	PARAM_NORMAL = 0,
	PARAM_READONLY = 1,
} param_edit_t;

typedef struct
{
	param_type_t type;
	param_edit_t editability;
	uint16_t address;
} param_t;

#define IO_GAP_TIME 10ms
#define EXECUTE_COMMAND_INDEX 15
#define COMMAND_REACH_SIGNAL "reach_15"
#define REVERSE(x) ((x << 16) & 0xFFFF0000) | ((x >> 16) & 0x0000FFFF)

inline void init_parameters(std::map<std::string, param_t>& params)
{
	params["current_command_position"] =		{ PARAM_FLOAT, PARAM_NORMAL, 4902 };
	params["current_command_velocity"] =		{ PARAM_FLOAT, PARAM_NORMAL, 4904 };
	params["current_command_acceleration"] =	{ PARAM_FLOAT, PARAM_NORMAL, 4906 };
	params["current_command_deacceleration"] =	{ PARAM_FLOAT, PARAM_NORMAL, 4908 };

	params["current_position"] = { PARAM_FLOAT, PARAM_READONLY, 0 };
	params["current_velocity"] = { PARAM_FLOAT, PARAM_READONLY, 2 };
	params["control_torque"] = { PARAM_FLOAT, PARAM_NORMAL, 4 };
	params["error"] = { PARAM_INT32, PARAM_READONLY, 6 };
	params["current_force_sensor"] = { PARAM_FLOAT, PARAM_READONLY, 18 };

	params["io_in_go_home"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1401 };
	params["io_in_error_reset"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1402 };
	params["io_in_start"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1403 };
	params["io_in_servo"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1404 };
	params["io_in_stop"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1405 };
	params["io_in_force_reset"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1424 };

	params["io_in_save_parameters"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1440 };
	params["io_in_load_parameters"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1441 };
	
	params["io_in_save_positions"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1442 };
	params["io_in_load_positions"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1443 };

	params["io_out_gone_home"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1501 };
	
	params["command_address"] = { PARAM_INT32, PARAM_NORMAL, 5000 };
	params["selected_command_index"] = { PARAM_INT32, PARAM_NORMAL, 4001 };

	params["io_out_reach_15"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1521};
	params["io_out_moving"] = { PARAM_BOOLEAN, PARAM_NORMAL, 1505};

	params["limit_pos"] =		{ PARAM_FLOAT, PARAM_NORMAL, 74 };
	params["limit_neg"] =		{ PARAM_FLOAT, PARAM_NORMAL, 72 };

	params["hardware_direct_output"] =		{ PARAM_INT32, PARAM_NORMAL, 158 };
	params["hardware_direct_input"] =		{ PARAM_INT32, PARAM_NORMAL, 160 };
}

#endif
