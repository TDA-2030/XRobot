#ifndef _RMAXIS_H_
#define _RMAXIS_H_

#include <memory>
#include <chrono>
#include <map>
#include <thread>
#include "Modbus.h"
#include "Config.h"

typedef struct {
	int32_t major;
	int32_t minor;
	int32_t build;
	int32_t type;
} version_t;

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

typedef struct
{
	Modbus* handle;
	int reference_count;
} modbus_device_t;

class RMAxis
{
public:
	~RMAxis();
	version_t get_version();

private:
	RMAxis(std::string device, bool is_modbus_rtu, int slave_id);

	// io
public:
	void set_input_signal(std::string signal, bool level);
	bool get_output_signal(std::string signal);

	// motion
public:
	void config_motion(float velocity, float acceleration, float deacceleration);
	void move_to(float position);

	void go_home();
	void move_absolute(
		float position, float velocity,
		float acceleration, float deacceleration, float band);
	void move_relative(
		float position, float velocity,
		float acceleration, float deacceleration, float band);
	void push(
		float force,
		float distance, float velocity);
	void precise_push(
		float force,
		float distance, float velocity,
		float force_band, uint32_t force_check_time);

	bool is_moving();
	bool is_reached();
	bool is_push_empty();

	// command
public:
	void set_command(int index, command_t command);
	command_t get_command(int index);
	void execute_command(command_t command);
	void trig_command(int index);
	void load_commands();
	void save_commands();

	// property
public:
	float position();
	float velocity();
	float torque();
	float force_sensor();
	uint32_t error_code();

	// parameters
public:
	void get_parameter(const char* name, void* data);
	void set_parameter(const char* name, void* data);

	template<typename T> inline
		T get_parameter(const char* name)
	{
		T result;
		get_parameter(name, &result);
		return result;
	}

	template<typename T, typename = typename std::enable_if<!std::is_same<T, void*>::value>::type > inline
		void set_parameter(const char* name, T value)
	{
		set_parameter(name, (void*)&value);
	}

	void load_parameters();
	void save_parameters();

	// misc
public:
	void reset_error();
	void set_servo_on_off(bool on_off);
	void stop();
	void soft_reset();

protected:
	std::map<int, bool> command_edited;

private:
	std::map<std::string, param_t> param;

	std::string device;
	static std::map<std::string, modbus_device_t> modbus_device;
	bool is_modbus_rtu;
	int slave_id;

public:
	static RMAxis* create_rmaxis_modbus_rtu(std::string device, int baudrate, uint8_t slave_id);
	static RMAxis* create_rmaxis_modbus_tcp(std::string address, int port, uint8_t slave_id);
	static void destroy_rmaxis(RMAxis* axis);
};

#endif
