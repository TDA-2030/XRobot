#include "wrapper.h"
#include <msclr\marshal_cppstd.h>
#include "../sdk_common/RMAxis.h"

using namespace motormaster;
using namespace msclr::interop;
using namespace System;

template<typename class_type, typename ret_type, typename ...arg_type>
inline ret_type safe_call(class_type* instance, ret_type(class_type::* method)(arg_type...), arg_type... args) {
	try {
		return (instance->*method)(std::forward<arg_type>(args)...);
	}
	catch (std::exception ex)
	{
		throw gcnew System::Exception(marshal_as<String^>(ex.what()));
	}
	return ret_type();
}

Axis^ Axis::CreateModbusRtu(String^ device, int baudrate, int slave_id)
{
	try {
		auto ctx = RMAxis::create_rmaxis_modbus_rtu(marshal_as<std::string>(device), baudrate, slave_id);
		auto axis = gcnew Axis();
		axis->ctx = ctx;
		return axis;
	}
	catch (std::exception ex)
	{
		throw gcnew System::Exception(marshal_as<String^>(ex.what()));
	}
}
Axis^ Axis::CreateModbusTcp(String^ address, int port, int slave_id)
{
	try {
		auto ctx = RMAxis::create_rmaxis_modbus_tcp(marshal_as<std::string>(address), port, slave_id);
		auto axis = gcnew Axis();
		axis->ctx = ctx;
		return axis;
	}
	catch (std::exception ex)
	{
		throw gcnew System::Exception(marshal_as<String^>(ex.what()));
	}
}
void Axis::Destroy(Axis^ axis)
{
	RMAxis::destroy_rmaxis((RMAxis*)axis->ctx);
}

// io
void Axis::SetInputSignal(String^ signal, bool level)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::set_input_signal, marshal_as<std::string>(signal), level);
}
bool Axis::GetOutputSignal(String^ signal)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::get_output_signal, marshal_as<std::string>(signal));
}

// motion
void Axis::ConfigMotion(float velocity, float acceleration, float deacceleration)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::config_motion, velocity, acceleration, deacceleration);
}
void Axis::MoveTo(float position)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::move_to, position);
}

void Axis::GoHome()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::go_home);
}
void Axis::MoveAbsolute(
	float position, float velocity,
	float acceleration, float deacceleration, float band)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::move_absolute, position, velocity, acceleration, deacceleration, band);
}
void Axis::MoveRelative(
	float position, float velocity,
	float acceleration, float deacceleration, float band)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::move_relative, position, velocity, acceleration, deacceleration, band);
}
void Axis::Push(
	float force,
	float distance, float velocity)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::push, force, distance, velocity);
}
void Axis::PrecisePush(
	float force,
	float distance, float velocity,
	float force_band, unsigned int force_check_time)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::precise_push, force, distance, velocity, force_band, force_check_time);
}

bool Axis::IsMoving()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::is_moving);
}
bool Axis::IsReached()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::is_reached);
}
bool Axis::IsPushEmpty()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::is_push_empty);
}

// command
void Axis::SetCommand(int index, Command command)
{
	command_t _command;
	_command.type = (command_type_t)command.Type; // marshal_as<command_type_t>(command.Type);
	_command.position = command.Position;
	_command.velocity = command.Velocity;
	_command.acceleration = command.Acceleration;
	_command.deacceleration = command.Deacceleration;
	_command.band = command.Band;
	_command.push_force = command.PushForce;
	_command.push_distance = command.PushDistance;
	_command.delay = command.Delay;
	_command.next_command_index = command.NextCommandIndex;

	safe_call(((RMAxis*)this->ctx), &RMAxis::set_command, index, _command);
}
Command Axis::GetCommand(int index)
{
	Command command;
	auto _command = safe_call(((RMAxis*)this->ctx), &RMAxis::get_command, index);

	command.Type = (CommandType)((int16_t)_command.type);
	command.Position = _command.position;
	command.Velocity = _command.velocity;
	command.Acceleration = _command.acceleration;
	command.Deacceleration = _command.deacceleration;
	command.Band = _command.band;
	command.PushForce = _command.push_force;
	command.PushDistance = _command.push_distance;
	command.Delay = _command.delay;
	command.NextCommandIndex = _command.next_command_index;

	return command;
}
void Axis::ExecuteCommand(Command command)
{
	command_t _command;
	_command.type = (command_type_t)command.Type;
	_command.position = command.Position;
	_command.velocity = command.Velocity;
	_command.acceleration = command.Acceleration;
	_command.deacceleration = command.Deacceleration;
	_command.band = command.Band;
	_command.push_force = command.PushForce;
	_command.push_distance = command.PushDistance;
	_command.delay = command.Delay;
	_command.next_command_index = command.NextCommandIndex;

	safe_call(((RMAxis*)this->ctx), &RMAxis::execute_command, _command);
}
void Axis::TrigCommand(int index)
{
	safe_call(((RMAxis*)this->ctx), &RMAxis::trig_command, index);
}
void Axis::LoadCommands()
{
	safe_call(((RMAxis*)this->ctx), &RMAxis::load_commands);
}
void Axis::SaveCommands()
{
	safe_call(((RMAxis*)this->ctx), &RMAxis::save_commands);
}

// property
float Axis::Position()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::position);
}
float Axis::Velocity()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::velocity);
}
float Axis::Torque()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::torque);
}
float Axis::ForceSensor()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::force_sensor);
}
int Axis::ErrorCode()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::error_code);
}

// parameters
void Axis::LoadParameters()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::load_parameters);
}
void Axis::SaveParameters()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::save_parameters);
}

// misc
void Axis::ResetError()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::reset_error);
}
void Axis::SetServoOnOff(bool on_off)
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::set_servo_on_off, on_off);
}
void Axis::Stop()
{
	return safe_call(((RMAxis*)this->ctx), &RMAxis::stop);
}