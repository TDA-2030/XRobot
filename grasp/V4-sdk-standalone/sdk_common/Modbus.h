#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

class Modbus
{
public:
	Modbus(std::string device_name, void* ctx);
	~Modbus();

public:
	// legacy api
	std::vector<uint8_t> read_inputs(int slave, uint16_t address, uint16_t quantity);
	std::vector<uint8_t> read_coils(int slave, uint16_t address, uint16_t quantity);
	void write_coils(int slave, uint16_t address, std::vector<uint8_t> data);
	void write_coil(int slave, uint16_t address, uint8_t data);

	std::vector<uint16_t> read_input_registers(int slave, uint16_t address, uint16_t quantity);
	std::vector<uint16_t> read_holding_registers(int slave, uint16_t address, uint16_t quantity);
	void write_registers(int slave, uint16_t address, std::vector<uint16_t> data);
	void write_register(int slave, uint16_t address, uint16_t data);


	// config
	void set_slave(int slave);

	void set_timing(uint32_t timeout_ms, uint32_t max_retry);

	// helper
	inline int32_t read_holding_int32(int slave, uint16_t address)
	{
		auto value = this->read_holding_registers(slave, address, 2);
		return (int32_t)((value[0] << 16) | (value[1]));
	}
	inline void write_holding_int32(int slave, uint16_t address, int32_t value)
	{
		std::vector<uint16_t> data = { (uint16_t)(value >> 16), (uint16_t)(value) };
		this->write_registers(slave, address, data);
	}

	inline float read_holding_float(int slave, uint16_t address)
	{
		return this->read_holding_int32(slave, address) / 1000.f;
	}
	inline void write_holding_float(int slave, uint16_t address, float value)
	{
		return this->write_holding_int32(slave, address, (int32_t)(value * 1000.f));
	}

	inline bool read_holding_bool(int slave, uint16_t address)
	{
		auto value = this->read_coils(slave, address, 1);
		return value[0] != 0;
	}
	inline void write_holding_bool(int slave, uint16_t address, bool value)
	{
		return this->write_coil(slave, address, value ? 1 : 0);
	}

	inline int32_t read_input_int32(int slave, uint16_t address)
	{
		auto value = this->read_input_registers(slave, address, 2);
		return (int32_t)((value[0] << 16) | (value[1]));
	}
	inline float read_input_float(int slave, uint16_t address)
	{
		return this->read_input_int32(slave, address) / 1000.f;
	}
	inline bool read_input_bool(int slave, uint16_t address)
	{
		auto value = this->read_inputs(slave, address, 1);
		return value[0] != 0;
	}

public:
	void* ctx;
	std::string device;
	uint32_t timeout_ms;
	uint32_t max_retry;
};

#endif