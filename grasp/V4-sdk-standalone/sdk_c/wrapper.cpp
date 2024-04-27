#include "motormaster.h"

#include <map>
#include <thread>
#include <exception>
#include <functional>
#include <type_traits>

std::map<std::thread::id, std::exception> exceptions;
std::map<std::thread::id, bool> exceptions_read;

template<typename class_type, typename ret_type, typename ...arg_type>
inline ret_type safe_call(class_type* instance, ret_type(class_type::* method)(arg_type...), arg_type... args) {
	try {
		return (instance->*method)(std::forward<arg_type>(args)...);
	}
	catch (std::exception ex)
	{
		auto thread_id = std::this_thread::get_id();
		exceptions_read[thread_id] = false;
		exceptions[thread_id] = ex;
	}
	return ret_type();
}

rm_handle_t rm_create_modbus_rtu(const char* device, int baudrate, uint8_t slave_id)
{
	try {
		return RMAxis::create_rmaxis_modbus_rtu(device, baudrate, slave_id);
	}
	catch (std::exception ex)
	{
		auto thread_id = std::this_thread::get_id();
		exceptions_read[thread_id] = false;
		exceptions[thread_id] = ex;
	}
	return nullptr;
}
rm_handle_t rm_create_modbus_tcp(const char* address, int port, uint8_t slave_id)
{
	try {
		return RMAxis::create_rmaxis_modbus_tcp(address, port, slave_id);
	}
	catch (std::exception ex)
	{
		auto thread_id = std::this_thread::get_id();
		exceptions_read[thread_id] = false;
		exceptions[thread_id] = ex;
	}
	return nullptr;
}

void rm_destroy(rm_handle_t handle)
{
	RMAxis::destroy_rmaxis((RMAxis*)handle);
}

const char* rm_get_exception(rm_handle_t handle)
{
	auto thread_id = std::this_thread::get_id();
	if (exceptions.count(thread_id) == 0)
	{
		return nullptr;
	}
	else if (exceptions_read[thread_id])
	{
		return nullptr;
	}
	else
	{
		exceptions_read[thread_id] = true;
		return exceptions[thread_id].what();
	}
}



void rm_set_input_signal(rm_handle_t handle, const char* signal, bool level)
{
	return safe_call((RMAxis*)handle, &RMAxis::set_input_signal, std::string(signal), level);
}
bool rm_get_output_signal(rm_handle_t handle, const char* signal)
{
	return safe_call((RMAxis*)handle, &RMAxis::get_output_signal, std::string(signal));
}

// motion
void rm_config_motion(rm_handle_t handle, float velocity, float acceleration, float deacceleration)
{
	return safe_call((RMAxis*)handle, &RMAxis::config_motion, velocity, acceleration, deacceleration);
}
void rm_move_to(rm_handle_t handle, float position)
{
	return safe_call((RMAxis*)handle, &RMAxis::move_to, position);
}

void rm_go_home(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::go_home);
}
void rm_move_absolute(
	rm_handle_t handle,
	float position, float velocity,
	float acceleration, float deacceleration, float band)
{
	return safe_call((RMAxis*)handle, &RMAxis::move_absolute, position, velocity, acceleration, deacceleration, band);
}
void rm_move_relative(
	rm_handle_t handle,
	float position, float velocity,
	float acceleration, float deacceleration, float band)
{
	return safe_call((RMAxis*)handle, &RMAxis::move_relative, position, velocity, acceleration, deacceleration, band);
}
void rm_push(
	rm_handle_t handle,
	float force,
	float distance, float velocity)
{
	return safe_call((RMAxis*)handle, &RMAxis::push, force, distance, velocity);
}
void rm_precise_push(
	rm_handle_t handle,
	float force,
	float distance, float velocity,
	float force_band, uint32_t force_check_time)
{
	return safe_call((RMAxis*)handle, &RMAxis::precise_push, force, distance, velocity, force_band, force_check_time);
}

bool rm_is_moving(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::is_moving);
}
bool rm_is_reached(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::is_reached);
}
bool rm_is_push_empty(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::is_push_empty);
}

// command
void rm_set_command(rm_handle_t handle, int index, command_t command)
{
	return safe_call((RMAxis*)handle, &RMAxis::set_command, index, command);
}
command_t rm_get_command(rm_handle_t handle, int index)
{
	return safe_call((RMAxis*)handle, &RMAxis::get_command, index);
}
void rm_execute_command(rm_handle_t handle, command_t command)
{
	return safe_call((RMAxis*)handle, &RMAxis::execute_command, command);
}
void rm_trig_command(rm_handle_t handle, int index)
{
	return safe_call((RMAxis*)handle, &RMAxis::trig_command, index);
}
void rm_load_commands(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::load_commands);
}
void rm_save_commands(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::save_commands);
}

// property
float rm_position(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::position);
}
float rm_velocity(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::velocity);
}
float rm_torque(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::torque);
}
float rm_force_sensor(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::force_sensor);
}
uint32_t rm_error_code(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::error_code);
}

void rm_load_parameters(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::load_parameters);
}
void rm_save_parameters(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::save_parameters);
}

// misc
void rm_reset_error(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::reset_error);
}
void rm_set_servo_on_off(rm_handle_t handle, bool on_off)
{
	return safe_call((RMAxis*)handle, &RMAxis::set_servo_on_off, on_off);
}
void rm_stop(rm_handle_t handle)
{
	return safe_call((RMAxis*)handle, &RMAxis::stop);
}
