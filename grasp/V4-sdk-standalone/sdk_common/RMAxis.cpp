#include "RMAxis.h"
#include "Exception.h"
#include <map>
#include <mutex>

extern "C"
{
#include "libmodbus/modbus.h"
}

using namespace std::chrono_literals;

std::map<std::string, std::mutex> rmaxis_device_lock;
std::map<std::string, modbus_device_t> RMAxis::modbus_device;

RMAxis* RMAxis::create_rmaxis_modbus_rtu(std::string device, int baudrate, uint8_t slave_id)
{
	if (modbus_device.count(device) == 0)
	{
		auto ctx = modbus_new_rtu(device.c_str(), baudrate, 'N', 8, 1);
		modbus_set_response_timeout((modbus_t*)ctx, 0, 100 * 1000);

		if (!ctx)
			NewException("ModbusRTU %s:Unable to allocate modbus rtu context",
				device);

		if (modbus_connect((modbus_t*)ctx) == -1) {
			modbus_free((modbus_t*)ctx);
			NewException("ModbusRTU %s:Connection failed: %s\n",
				device, modbus_strerror(errno));
		}

		modbus_device[device].handle = new Modbus(device.c_str(), ctx);
		modbus_device[device].reference_count = 1;
	}
	else
		modbus_device[device].reference_count++;

	return new RMAxis(device, true, slave_id + 1);
}
RMAxis* RMAxis::create_rmaxis_modbus_tcp(std::string address, int port, uint8_t slave_id)
{
	if (modbus_device.count(address) == 0)
	{
		auto ctx = modbus_new_tcp(address.c_str(), port);
		modbus_set_response_timeout((modbus_t*)ctx, 0, 100 * 1000);
		modbus_set_error_recovery((modbus_t*)ctx, MODBUS_ERROR_RECOVERY_LINK);
		
		if (!ctx)
			NewException("ModbusTCP %s:%d Unable to allocate modbus tcp context",
				address, port);

		if (modbus_connect((modbus_t*)ctx) == -1) {
			modbus_free((modbus_t*)ctx);
			NewException("ModbusTCP %s:%d Connection failed: %s\n",
				address, port, modbus_strerror(errno));
		}

		modbus_device[address].handle = new Modbus(address.c_str(), ctx);
		modbus_device[address].reference_count = 1;
	}
	else
		modbus_device[address].reference_count++;

	return new RMAxis(address, false, slave_id + 1);
}

void RMAxis::destroy_rmaxis(RMAxis* axis)
{
	auto device = axis->device;

	modbus_device[axis->device].reference_count--;
	if (modbus_device[axis->device].reference_count == 0)
	{
		modbus_close((modbus_t*)((Modbus*)modbus_device[axis->device].handle)->ctx);
		modbus_free((modbus_t*)((Modbus*)modbus_device[axis->device].handle)->ctx);
		delete (Modbus*)modbus_device[axis->device].handle;
		modbus_device.erase(axis->device);
	}

	delete axis;
}

RMAxis::RMAxis(std::string device, bool is_modbus_rtu, int slave_id)
	:device(device), is_modbus_rtu(is_modbus_rtu), slave_id(slave_id)
{
	init_parameters(this->param);
}

RMAxis::~RMAxis()
{

}

version_t RMAxis::get_version()
{
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);

	auto version_major = this->modbus_device[device].handle->read_input_int32(slave_id, 8);
	auto version_minor = this->modbus_device[device].handle->read_input_int32(slave_id, 10);
	auto version_build = this->modbus_device[device].handle->read_input_int32(slave_id, 12);
	auto version_type = this->modbus_device[device].handle->read_input_int32(slave_id, 14);

	return version_t{ version_major, version_minor, version_build, version_type };
}

void RMAxis::set_input_signal(std::string signal, bool level)
{
	auto param_name = "io_in_" + std::string(signal);
	this->set_parameter<bool>(param_name.c_str(), level);
}

bool RMAxis::get_output_signal(std::string signal)
{
	auto param_name = "io_out_" + std::string(signal);
	auto result = this->get_parameter<bool>(param_name.c_str());
	return result;
}

void RMAxis::config_motion(float velocity, float acceleration, float deacceleration)
{
	this->set_parameter<float>("current_command_velocity", velocity);
	this->set_parameter<float>("current_command_acceleration", acceleration);
	this->set_parameter<float>("current_command_deacceleration", deacceleration);
}

void RMAxis::move_to(float position)
{
	this->set_parameter<float>("current_command_position", position);
}


void RMAxis::go_home()
{
	this->set_input_signal("go_home", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("go_home", true);
}

void RMAxis::move_absolute(float position, float velocity, float acceleration, float deacceleration, float band)
{
	command_t command;
	command.type = COMMAND_MOVE_ABSOLUTE;
	command.position = position;
	command.velocity = velocity;
	command.acceleration = acceleration;
	command.deacceleration = deacceleration;
	command.band = band;
	command.push_force = 0;
	command.push_distance = 0;
	command.delay = 0;
	command.next_command_index = -1;

	this->execute_command(command);
}

void RMAxis::move_relative(
	float position, float velocity,
	float acceleration, float deacceleration, float band)
{
	command_t command;
	command.type = COMMAND_MOVE_RELATIVE;
	command.position = position;
	command.velocity = velocity;
	command.acceleration = acceleration;
	command.deacceleration = deacceleration;
	command.band = band;
	command.push_force = 0;
	command.push_distance = 0;
	command.delay = 0;
	command.next_command_index = -1;

	this->execute_command(command);
}

void RMAxis::push(float force, float distance, float velocity)
{
	command_t command;
	command.type = COMMAND_PUSH;
	command.position = 0;
	command.velocity = velocity;
	command.acceleration = 0;
	command.deacceleration = 0;
	command.band = 0;
	command.push_force = force;
	command.push_distance = distance;
	command.delay = 0;
	command.next_command_index = -1;

	this->execute_command(command);
}

void RMAxis::precise_push(float force, float distance, float velocity, float force_band, uint32_t force_check_time)
{
	command_t command;
	command.type = COMMAND_PRECISE_PUSH;
	command.position = 0;
	command.velocity = velocity;
	command.acceleration = 0;
	command.deacceleration = 0;
	command.band = force_band;
	command.push_force = force;
	command.push_distance = distance;
	command.delay = force_check_time;
	command.next_command_index = -1;

	this->execute_command(command);
}

bool RMAxis::is_moving()
{
	return this->get_output_signal("moving");
}

bool RMAxis::is_reached()
{
	return this->get_output_signal(COMMAND_REACH_SIGNAL);
}

bool RMAxis::is_push_empty()
{
	return !this->is_moving();
}

void RMAxis::set_command(int index, command_t command)
{
	command_edited[index] = true;
	
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);

	int32_t command_buffer[10] = { 0 };
	command_buffer[0] = REVERSE(command.type);
	command_buffer[1] = REVERSE((int)(command.position * 1000));
	command_buffer[2] = REVERSE((int)(command.velocity * 1000));
	command_buffer[3] = REVERSE((int)(command.acceleration * 1000));
	command_buffer[4] = REVERSE((int)(command.deacceleration * 1000));
	command_buffer[5] = REVERSE((int)(command.band * 1000));
	command_buffer[6] = REVERSE((int)(command.push_force * 1000));
	command_buffer[7] = REVERSE((int)(command.push_distance * 1000));
	command_buffer[8] = REVERSE((int)(command.delay));
	command_buffer[9] = REVERSE((int)(command.next_command_index));

	std::vector<uint16_t> buffer(20);
	memcpy(&buffer[0], &command_buffer, 40);

	this->modbus_device[device].handle->write_registers(
		slave_id, 
		this->param["command_address"].address + index * 20, 
		buffer);

}

command_t RMAxis::get_command(int index)
{
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);

	auto buffer = this->modbus_device[device].handle->read_holding_registers(
		slave_id,
		this->param["command_address"].address + index * 20,
		20
	);

	int32_t* command_buffer = (int32_t*)&buffer[0];

	command_t command;
	command.type = (command_type_t)(REVERSE(command_buffer[0]));
	command.position = ((int32_t)(REVERSE(command_buffer[1]))) / 1000.f;
	command.velocity = ((int32_t)(REVERSE(command_buffer[2]))) / 1000.f;
	command.acceleration = ((int32_t)(REVERSE(command_buffer[3]))) / 1000.f;
	command.deacceleration = ((int32_t)(REVERSE(command_buffer[4]))) / 1000.f;
	command.band = ((int32_t)(REVERSE(command_buffer[5]))) / 1000.f;
	command.push_force = ((int32_t)(REVERSE(command_buffer[6]))) / 1000.f;
	command.push_distance = ((int32_t)(REVERSE(command_buffer[7]))) / 1000.f;
	command.delay = ((int32_t)(REVERSE(command_buffer[8])));
	command.next_command_index = ((int32_t)REVERSE(command_buffer[9]));

	return command;
}

void RMAxis::execute_command(command_t command)
{
	this->set_command(EXECUTE_COMMAND_INDEX, command);
	this->trig_command(EXECUTE_COMMAND_INDEX);
	std::this_thread::sleep_for(IO_GAP_TIME);
}

void RMAxis::trig_command(int index)
{
	this->set_parameter<int>("selected_command_index", index);
	this->set_input_signal("start", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("start", true);
}

void RMAxis::load_commands()
{
	this->set_input_signal("load_positions", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("load_positions", true);
}

void RMAxis::save_commands()
{
	for (auto& pair : command_edited)
	{
		if (pair.second)
		{
			this->set_parameter<int>("selected_command_index", pair.first);
			this->set_input_signal("save_positions", false);
			std::this_thread::sleep_for(IO_GAP_TIME);
			this->set_input_signal("save_positions", true);
			std::this_thread::sleep_for(IO_GAP_TIME);
			pair.second = false;
		}
	}
}

float RMAxis::position()
{
	return this->get_parameter<float>("current_position");
}

float RMAxis::velocity()
{
	return this->get_parameter<float>("current_velocity");
}

float RMAxis::torque()
{
	return this->get_parameter<float>("control_torque");
}

float RMAxis::force_sensor()
{
	return this->get_parameter<float>("current_force_sensor");
}

uint32_t RMAxis::error_code()
{
	return this->get_parameter<uint32_t>("error");
}

void RMAxis::get_parameter(const char* name, void* data)
{
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);

	if (this->param.count(name) == 0)
		NewException("parameter %s does not exist", name);

	auto param = this->param[name];

	auto address = param.address;

	if (param.editability == PARAM_READONLY)
	{
		switch (param.type)
		{
		case PARAM_BOOLEAN:
			*(bool*)data = modbus_device[device].handle->read_input_bool(slave_id, address);
			break;
		case PARAM_ENUM:
		{
			auto result = modbus_device[device].handle->read_input_registers(slave_id, address, 1);
			*(int16_t*)data = result.front();
		}
		break;
		case PARAM_INT32:
			*(int32_t*)data = modbus_device[device].handle->read_input_int32(slave_id, address);
			break;
		case PARAM_FLOAT:
			*(float*)data = modbus_device[device].handle->read_input_float(slave_id, address);
			break;
		default:
			NewException("parameter %s has unknown data type %d", name, param.type);
		}
	}
	else
	{
		switch (param.type)
		{
		case PARAM_BOOLEAN:
			*(bool*)data = modbus_device[device].handle->read_holding_bool(slave_id, address);
			break;
		case PARAM_ENUM:
		{
			auto result = modbus_device[device].handle->read_holding_registers(slave_id, address, 1);
			*(int16_t*)data = result.front();
		}
		break;
		case PARAM_INT32:
			*(int32_t*)data = modbus_device[device].handle->read_holding_int32(slave_id, address);
			break;
		case PARAM_FLOAT:
			*(float*)data = modbus_device[device].handle->read_holding_float(slave_id, address);
			break;
		default:
			NewException("parameter %s has unknown data type %d", name, param.type);
		}
	}
}

void RMAxis::set_parameter(const char* name, void* data)
{
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);

	auto param = this->param[name];

	if (param.editability == PARAM_READONLY)
		NewException("parameter %s is read only", name);

	else
	{
		auto address = param.address;

		switch (param.type)
		{
		case PARAM_BOOLEAN:
			modbus_device[device].handle->write_holding_bool(slave_id, address, *(bool*)data);
			break;
		case PARAM_ENUM:
			modbus_device[device].handle->write_register(slave_id, address, *(uint16_t*)data);
			break;
		case PARAM_INT32:
			modbus_device[device].handle->write_holding_int32(slave_id, address, *(int32_t*)data);
			break;
		case PARAM_FLOAT:
			modbus_device[device].handle->write_holding_float(slave_id, address, *(float*)data);
			break;
		default:
			NewException("parameter %s has unknown data type %d", name, param.type);
		}
	}
}

void RMAxis::load_parameters()
{
	this->set_input_signal("load_parameters", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("load_parameters", true);
}

void RMAxis::save_parameters()
{
	this->set_input_signal("save_parameters", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("save_parameters", true);
}

void RMAxis::reset_error()
{
	this->set_input_signal("error_reset", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("error_reset", true);
}

void RMAxis::set_servo_on_off(bool on_off)
{
	this->set_input_signal("servo", on_off);
}

void RMAxis::stop()
{
	this->set_input_signal("stop", false);
	std::this_thread::sleep_for(IO_GAP_TIME);
	this->set_input_signal("stop", true);
}

void RMAxis::soft_reset() 
{
	std::lock_guard<std::mutex> lock_guard(rmaxis_device_lock[device]);
	modbus_device[device].handle->write_holding_int32(slave_id, 186, 0x22205682);
}
