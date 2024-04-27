#ifndef _MOTORMASTER_H_
#define _MOTORMASTER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#ifdef RM_API_EXPORTS
#define RM_API __declspec(dllexport)
#else
#define RM_API __declspec(dllimport)
#endif
#else
#ifdef RM_API_EXPORTS
#define RM_API __attribute__ ((visibility ("default")))
#else
#define RM_API 
#endif
#endif

#ifdef RM_API_EXPORTS

#include "../sdk_common/RMAxis.h"

#else

typedef enum {
	COMMAND_NONE = 0,
	COMMAND_GO_HOME = 1,
	COMMAND_DELAY = 2,
	COMMAND_MOVE_ABSOLUTE = 3,
	COMMAND_PUSH = 4,
	COMMAND_MOVE_RELATIVE = 5,
	COMMAND_PRECISE_PUSH = 6
} command_type_t;

typedef struct {
	command_type_t type;
	float position;
	float velocity;
	float acceleration;
	float deacceleration;
	float band;
	float push_force;
	float push_distance;
	int32_t delay;
	int32_t next_command_index;
} command_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void* rm_handle_t;

RM_API rm_handle_t rm_create_modbus_rtu(const char* device, int baudrate, uint8_t slave_id);
RM_API rm_handle_t rm_create_modbus_tcp(const char* address, int port, uint8_t slave_id);
RM_API void rm_destroy(rm_handle_t handle);

RM_API const char* rm_get_exception(rm_handle_t handle);

// io
RM_API void rm_set_input_signal(rm_handle_t handle, const char* signal, bool level);
RM_API bool rm_get_output_signal(rm_handle_t handle, const char* signal);

// motion
RM_API void rm_config_motion(rm_handle_t handle, float velocity, float acceleration, float deacceleration);
RM_API void rm_move_to(rm_handle_t handle, float position);

RM_API void rm_go_home(rm_handle_t handle);
RM_API void rm_move_absolute(
	rm_handle_t handle,
	float position, float velocity,
	float acceleration, float deacceleration, float band);
RM_API void rm_move_relative(
	rm_handle_t handle,
	float position, float velocity,
	float acceleration, float deacceleration, float band);
RM_API void rm_push(
	rm_handle_t handle,
	float force,
	float distance, float velocity);
RM_API void rm_precise_push(
	rm_handle_t handle,
	float force,
	float distance, float velocity,
	float force_band, uint32_t force_check_time);

RM_API bool rm_is_moving(rm_handle_t handle);
RM_API bool rm_is_reached(rm_handle_t handle);
RM_API bool rm_is_push_empty(rm_handle_t handle);

// command
RM_API void rm_set_command(rm_handle_t handle, int index, command_t command);
RM_API command_t rm_get_command(rm_handle_t handle, int index);
RM_API void rm_execute_command(rm_handle_t handle, command_t command);
RM_API void rm_trig_command(rm_handle_t handle, int index);
RM_API void rm_load_commands(rm_handle_t handle);
RM_API void rm_save_commands(rm_handle_t handle);

// property
RM_API float rm_position(rm_handle_t handle);
RM_API float rm_velocity(rm_handle_t handle);
RM_API float rm_torque(rm_handle_t handle);
RM_API float rm_force_sensor(rm_handle_t handle);
RM_API uint32_t rm_error_code(rm_handle_t handle);

RM_API void rm_load_parameters(rm_handle_t handle);
RM_API void rm_save_parameters(rm_handle_t handle);

// misc
RM_API void rm_reset_error(rm_handle_t handle);
RM_API void rm_set_servo_on_off(rm_handle_t handle, bool on_off);
RM_API void rm_stop(rm_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif